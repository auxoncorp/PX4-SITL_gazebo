/*
 * GENERATED CODE, DO NOT EDIT
 *
 * Component:
 *   Name: gazebo
 *   ID: a8097269-0c33-49b1-9334-9a514d11509e
 *   Code hash: 108c8249d1c0785ed3f7afe2ac518da979bc4f2530315440ea5e44a4e6c40e60
 *   Instrumentation hash: 0bcbcf5f84f536bc7f5d2fcb248c2886ea04a60897372c2afc660c437a7dfac8
 */

#ifndef MODALITY_PROBE_GENERATED_IDENTIFIERS_H
#define MODALITY_PROBE_GENERATED_IDENTIFIERS_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Probes (sha3-256 81ac42e8e1f7b3fbaaedcca645cc8f4b2ee720d530064cdedab2ff3c0196c494)
 */

/*
 * Name: SIMULATOR
 * Description: Gazebo simulator plugin probe
 * Component ID: a8097269-0c33-49b1-9334-9a514d11509e
 * Tags: gazebo;gazebo-plugin;simulator;control-plane
 * Location: gazebo_probe_plugin.cpp:33
 */
#define SIMULATOR (1060744754UL)

/*
 * Events (sha3-256 5e5e227cbe8cc34a74273f02a28f9831c90caf4ecd47e3eb907d91b110f223f3)
 */

/*
 * Name: GROUND_TRUTH_ALTITUDE
 * Description: Ground truth altitude: [cm]
 * Component ID: a8097269-0c33-49b1-9334-9a514d11509e
 * Tags: gazebo;ground-truth
 * Payload type: i32
 * Location: gazebo_probe_plugin.cpp:162
 */
#define GROUND_TRUTH_ALTITUDE (1UL)

/*
 * Name: APPLY_IMPACT_FORCE
 * Description: Mutation impact force: [Newtons]
 * Component ID: a8097269-0c33-49b1-9334-9a514d11509e
 * Tags: gazebo;impact-force
 * Payload type: f32
 * Location: gazebo_probe_plugin.cpp:108
 */
#define APPLY_IMPACT_FORCE (2UL)

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MODALITY_PROBE_GENERATED_IDENTIFIERS_H */
