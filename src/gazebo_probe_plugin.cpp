#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math/Vector3.hh>
#include <Groundtruth.pb.h>
#include <Snapshot.pb.h>
#include <boost/lockfree/spsc_queue.hpp>

#include <modality_helpers.h>
#include <gazebo_component_definitions.h>
#include <mutators/impact_force.h>

using namespace std;
using namespace gazebo::common;

namespace gazebo
{
    typedef const boost::shared_ptr<const sensor_msgs::msgs::Groundtruth> GtPtr;
    const size_t Q_SIZE = 8;

    class ProbePlugin : public ModelPlugin
    {
        public: ProbePlugin() : ModelPlugin()
        {
            probe_report_socket_init(&report_socket);
            ctrl_msg_recvr = udp_control_message_receiver_new();
            assert(ctrl_msg_recvr != NULL);
            size_t err = udp_control_message_receiver_run(UDP_CONTROL_RECVR_GAZEBO_PLUGIN, ctrl_msg_recvr);
            assert(err == 0);

            err = MODALITY_PROBE_INIT(
                    &probe_storage[0],
                    sizeof(probe_storage),
                    SIMULATOR,
                    WALL_CLOCK_RESOLUTION_NS,
                    WALL_CLOCK_ID,
                    &next_persistent_sequence_id,
                    NULL, /* No user data needed for our next_persistent_sequence_id implementation */
                    &probe,
                    MODALITY_TAGS("gazebo", "gazebo-plugin", "simulator", "control-plane"),
                    "Gazebo simulator plugin probe");
            assert(err == MODALITY_PROBE_ERROR_OK);
            LOG_PROBE_INIT_W_RECVR(SIMULATOR, UDP_CONTROL_RECVR_GAZEBO_PLUGIN);

            (void) memset(&impact_force_state, 0, sizeof(impact_force_state));
            (void) memset(&impact_force_mutator, 0, sizeof(impact_force_mutator));

            impact_force_state.active = false;
            impact_force_mutator.state = (void*) &impact_force_state;
            impact_force_mutator.inject_mutation = &impact_force_inject_mutation;
            impact_force_mutator.clear_mutations = &impact_force_clear_mutations;
            impact_force_mutator.get_definition = &impact_force_get_definition;

            err = modality_probe_register_mutator(
                    probe,
                    &impact_force_mutator);
            assert(err == MODALITY_PROBE_ERROR_OK);

            send_probe_report(probe, report_socket, report_buffer, sizeof(report_buffer));
            report_timer.Start();
            mutator_announcement_timer.Start();
            assert(report_timer.GetRunning() == true);
            assert(mutator_announcement_timer.GetRunning() == true);
        }

        public: ~ProbePlugin()
        {
            send_probe_report(probe, report_socket, report_buffer, sizeof(report_buffer));

            const size_t err = udp_control_message_receiver_free(ctrl_msg_recvr);
            assert(err == 0);
            probe_report_socket_deinit(&report_socket);
        }

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            this->model = _parent;
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&ProbePlugin::OnUpdate, this));
            this->nspace.clear();
            this->node_handle = transport::NodePtr(new transport::Node());
            this->node_handle->Init(this->nspace);
            this->groundtruth_sub_topic = "/groundtruth";
            this->groundtruth_sub = this->node_handle->Subscribe(
                    "~/" + this->model->GetName() + this->groundtruth_sub_topic,
                    &ProbePlugin::GroundtruthCallback,
                    this);
            snapshot_pub = this->node_handle->Advertise<sensor_msgs::msgs::Snapshot>(
                    "~/" + this->model->GetName() + "/snapshot", 10);
        }

        public: void OnUpdate()
        {
            size_t err;

            err = udp_control_message_dequeue(
                    ctrl_msg_recvr,
                    control_msg_callback,
                    (void*) probe);
            assert(err == 0);

            ProcessSimulatorMessages();

            if(impact_force_state.active == true)
            {
                err = MODALITY_PROBE_RECORD_W_F32_W_TIME(
                        probe,
                        APPLY_IMPACT_FORCE,
                        impact_force_state.force,
                        ModalityTimeStampNs(),
                        MODALITY_TAGS("gazebo", "impact-force"),
                        "Mutation impact force: [Newtons]");
                assert(err == MODALITY_PROBE_ERROR_OK);

                assert(
                        impact_force_state.link >= LINK_ROTOR_0
                        && impact_force_state.link <= LINK_ROTOR_3);
                const char * const link_name = LINK_NAMES[impact_force_state.link];

                physics::LinkPtr link = this->model->GetLink(link_name);
                assert(link != NULL);

                link->AddRelativeForce(
                        ignition::math::Vector3d(
                            0,
                            0,
                            impact_force_state.force
                        )
                );

                PublishProbeSnapshot();

                impact_force_state.active = false;
            }

            SendProbeReport();
        }

        private: void ProcessSimulatorMessages()
        {
            size_t err;
            sensor_msgs::msgs::Groundtruth msg;

            size_t i;
            bool did_log_event = false;
            for(i = 0; i < Q_SIZE ; i += 1)
            {
                if(msg_q.pop(msg) == false)
                {
                    break;
                }

                // The mavlink interface uses mm, we're logging cm
                const int32_t gt_alt = msg.altitude() * 100;

                if(gt_alt != last_gt_alt)
                {
                    did_log_event = true;
                    last_gt_alt = gt_alt;
                    err = MODALITY_PROBE_RECORD_W_I32_W_TIME(
                            probe,
                            GROUND_TRUTH_ALTITUDE,
                            gt_alt,
                            sim_time_to_ns(msg.time_usec()),
                            MODALITY_TAGS("gazebo", "ground-truth"),
                            "Ground truth altitude: [cm]");
                    assert(err == MODALITY_PROBE_ERROR_OK);
                }
            }

            if(did_log_event == true)
            {
                PublishProbeSnapshot();
            }
        }

        private: void GroundtruthCallback(GtPtr& msg)
        {
            if(msg_q.push(*msg) == false)
            {
                gzwarn << "Groundtruth message queue full" << endl;
            }
        }

        private: void PublishProbeSnapshot()
        {
            size_t snapshot_size = 0;
            const size_t err = modality_probe_produce_snapshot_bytes(
                    probe,
                    &snapshot[0],
                    sizeof(snapshot),
                    &snapshot_size);
            assert(err == MODALITY_PROBE_ERROR_OK);
            assert(snapshot_size == sizeof(snapshot));

            sensor_msgs::msgs::Snapshot snapshot_msg;
            snapshot_msg.set_time_usec(this->model->GetWorld()->SimTime().Double() * 1e6);
            snapshot_msg.set_sequence_number(snapshot_seq_num);
            snapshot_msg.set_word0(u32_from_bytes(&snapshot[0]));
            snapshot_msg.set_word1(u32_from_bytes(&snapshot[4]));
            snapshot_msg.set_word2(u32_from_bytes(&snapshot[8]));
            snapshot_pub-> Publish(snapshot_msg);
            snapshot_seq_num += 1;
        }

        private: void SendProbeReport()
        {
            if(report_timer.GetElapsed() >= report_interval)
            {
                send_probe_report(probe, report_socket, report_buffer, sizeof(report_buffer));
                report_timer.Reset();
                report_timer.Start();
            }
            if(mutator_announcement_timer.GetElapsed() >= mutator_announcement_interval)
            {
                send_mutator_announcement(probe, report_socket, report_buffer, sizeof(report_buffer));
                mutator_announcement_timer.Reset();
                mutator_announcement_timer.Start();
            }
        }

        private: uint64_t ModalityTimeStampNs()
        {
            const uint64_t us = this->model->GetWorld()->SimTime().Double() * 1e6;
            return sim_time_to_ns(us);
        }

        private:
            physics::ModelPtr model;
            event::ConnectionPtr updateConnection;
            std::string nspace;
            std::string groundtruth_sub_topic;
            transport::NodePtr node_handle;

            transport::SubscriberPtr groundtruth_sub;
            boost::lockfree::spsc_queue<sensor_msgs::msgs::Groundtruth> msg_q{Q_SIZE};

            transport::PublisherPtr snapshot_pub;

            modality_probe *probe = MODALITY_PROBE_NULL_INITIALIZER;
            uint8_t probe_storage[PROBE_SIZE];
            uint8_t report_buffer[REPORT_SIZE];
            int report_socket = -1;
            Timer report_timer;
            Timer mutator_announcement_timer;
            const Time report_interval = Time(0, Time::MilToNano(REPORT_INTERVAL_MS));
            const Time mutator_announcement_interval = Time(0, Time::MilToNano(MUTATOR_ANNOUNCEMENT_INTERVAL_MS));

            udp_control_message_receiver *ctrl_msg_recvr = NULL;
            uint32_t snapshot_seq_num = 0;
            uint8_t snapshot[12];

            modality_mutation_interface impact_force_mutator;
            impact_force_state_s impact_force_state;

            int32_t last_gt_alt = 0;
    };

    GZ_REGISTER_MODEL_PLUGIN(ProbePlugin)
}
