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

namespace gazebo
{
    typedef const boost::shared_ptr<const sensor_msgs::msgs::Groundtruth> GtPtr;

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
                    GAZEBO_SIMULATOR,
                    WALL_CLOCK_RESOLUTION_NS,
                    WALL_CLOCK_ID,
                    &next_persistent_sequence_id,
                    NULL, /* No user data needed for our next_persistent_sequence_id implementation */
                    &probe,
                    MODALITY_TAGS("gazebo", "gazebo-plugin", "simulator", "control-plane"),
                    "Gazebo simulator plugin probe");
            assert(err == MODALITY_PROBE_ERROR_OK);
            LOG_PROBE_INIT_W_RECVR(GAZEBO_SIMULATOR, UDP_CONTROL_RECVR_GAZEBO_PLUGIN);

            send_probe_report(probe, report_socket, report_buffer, sizeof(report_buffer));
            last_report_time = common::Time::GetWallTime();
            snapshot_seq_num = 0;
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

            sensor_msgs::msgs::Groundtruth msg;
            while(msg_q.pop(msg) == true)
            {
                // The mavlink interface uses mm, we're logging cm
                const int32_t gt_alt = msg.altitude() * 100;

                if(gt_alt != last_gt_alt)
                {
                    err = MODALITY_PROBE_RECORD_W_I32(
                            probe,
                            GROUND_TRUTH_ALTITUDE,
                            gt_alt,
                            MODALITY_TAGS("gazebo", "ground-truth"),
                            "Ground truth altitude: [cm]");
                    assert(err == MODALITY_PROBE_ERROR_OK);
                }

                size_t snapshot_size = 0;
                err = modality_probe_produce_snapshot_bytes(
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

            common::Time report_interval =
                common::Time(0, common::Time::MilToNano(REPORT_INTERVAL_MS));
            common::Time now = common::Time::GetWallTime();
            if((now - last_report_time) >= report_interval)
            {
                send_probe_report(probe, report_socket, report_buffer, sizeof(report_buffer));

                last_report_time = common::Time::GetWallTime();
            }

        }

        private: void GroundtruthCallback(GtPtr& msg)
        {
            const bool status = msg_q.push(*msg);
            assert(status == true);
        }

        private:
            physics::ModelPtr model;
            event::ConnectionPtr updateConnection;
            std::string nspace;
            std::string groundtruth_sub_topic;
            transport::NodePtr node_handle;

            transport::SubscriberPtr groundtruth_sub;
            boost::lockfree::spsc_queue<sensor_msgs::msgs::Groundtruth> msg_q{64};

            transport::PublisherPtr snapshot_pub;

            modality_probe *probe = MODALITY_PROBE_NULL_INITIALIZER;
            uint8_t probe_storage[PROBE_SIZE];
            uint8_t report_buffer[REPORT_SIZE];
            int report_socket = -1;
            common::Time last_report_time;
            udp_control_message_receiver *ctrl_msg_recvr = NULL;
            uint32_t snapshot_seq_num = 0;
            uint8_t snapshot[12];

            int32_t last_gt_alt = 0;
    };

    GZ_REGISTER_MODEL_PLUGIN(ProbePlugin)
}
