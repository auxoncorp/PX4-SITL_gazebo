#ifndef IMPACT_FORCE_H
#define IMPACT_FORCE_H

#include <stdint.h>
#include <stdbool.h>

#include <modality/mutation_interface.h>

#ifdef __cplusplus
extern "C" {
#endif

// Based on the iris model
typedef enum
{
    LINK_ROTOR_0 = 0,
    LINK_ROTOR_1 = 1,
    LINK_ROTOR_2 = 2,
    LINK_ROTOR_3 = 3,
} impact_force_link_kind;

typedef struct
{
    bool active;        //  Single-shot mutator, auto-disables after force is applied
    float force;        // Vertical force magnitude and direction
    int32_t link;       // Link/location to apply the force
} impact_force_state_s;

extern const char *LINK_NAMES[4];

size_t impact_force_get_definition(
        void * const fi,
        const modality_mutation_definition ** const definition);

size_t impact_force_clear_mutations(
        void * const fi);

size_t impact_force_inject_mutation(
        void * const fi,
        const modality_mutation_param * const params,
        const size_t params_length);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* IMPACT_FORCE_H */

