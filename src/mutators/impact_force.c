#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "modality/probe.h"
#include "modality/mutation_interface.h"

#include "mutators/impact_force.h"

const char *LINK_NAMES[4] =
{
    [LINK_ROTOR_0] = "rotor_0",
    [LINK_ROTOR_1] = "rotor_1",
    [LINK_ROTOR_2] = "rotor_2",
    [LINK_ROTOR_3] = "rotor_3",
};

static const char *TAGS[3] =
{
    "gazebo",
    "single-shot",
    "impact-force",
};

static const modality_mutation_option_value_and_name LINK_VALUES_AND_NAMES[4] =
{
    [0] = { LINK_ROTOR_0, "ROTOR_0" },
    [1] = { LINK_ROTOR_1, "ROTOR_1" },
    [2] = { LINK_ROTOR_2, "ROTOR_2" },
    [3] = { LINK_ROTOR_3, "ROTOR_3" }
};

static const modality_mutation_parameter_definition PARAM_DEFS[2] =
{
    [0] =
    {
        .param_type =
        {
            .tag = MUTATION_PARAM_TYPE_ENUM,
            .body.enumeration =
            {
                .option_values_and_names = LINK_VALUES_AND_NAMES,
                .option_values_and_names_length = 4,
                .constraints =
                {
                    .minimum_effect_value = LINK_ROTOR_0,
                    .nominal_range = { .inclusive_start = LINK_ROTOR_0, .inclusive_end = LINK_ROTOR_3 },
                    .safety_range = { .inclusive_start = LINK_ROTOR_0, .inclusive_end = LINK_ROTOR_3 },
                    .hard_range = { .inclusive_start = LINK_ROTOR_0, .inclusive_end = LINK_ROTOR_3 },
                },
            },
        },
        .name = "link",
    },
    [1] =
    {
        .param_type =
        {
            .tag = MUTATION_PARAM_TYPE_F32,
            .body.f32 =
            {
                .minimum_effect_value = 0.0f,
                .nominal_range = { .inclusive_start = -1000.0f, .inclusive_end = 1000.0f },
                .safety_range = { .inclusive_start = -2000.0f, .inclusive_end = 2000.0f },
                .hard_range = { .inclusive_start = -3000.0f, .inclusive_end = 3000.0f },
            },
        },
        .name = "force",
    },
};

static const modality_mutation_definition MUT_DEF =
{
    .name = "impact-force-mutator",
    .params = PARAM_DEFS,
    .params_length = 2,
    .tags = TAGS,
    .tags_length = 3,
};

size_t impact_force_get_definition(
        void * const fi,
        const modality_mutation_definition ** const definition)
{
    assert(fi != NULL);
    (*definition) = &MUT_DEF;
    return 0;
}

size_t impact_force_clear_mutations(
        void * const fi)
{
    printf("Clearing '%s' mutations\n", MUT_DEF.name);

    assert(fi != NULL);
    impact_force_state_s * const state = (impact_force_state_s*) fi;
    state->active = false;

    return 0;
}

size_t impact_force_inject_mutation(
        void * const fi,
        const modality_mutation_param * const params,
        const size_t params_length)
{
    assert(fi != NULL);
    assert(params_length == MUT_DEF.params_length);

    printf(
            "Injecting '%s' mutations: '%s' = %d, '%s' = %.03f\n",
            MUT_DEF.name,
            MUT_DEF.params[0].name,
            (int) params[0].body.enum_selection.value,
            MUT_DEF.params[1].name,
            params[1].body.f32.value);

    impact_force_state_s * const state = (impact_force_state_s*) fi;

    assert(params[0].tag == MUTATION_PARAM_ENUM_SELECTION);
    state->link = params[0].body.enum_selection.value;

    assert(params[1].tag == MUTATION_PARAM_F32);
    state->force = params[1].body.f32.value;

    state->active = true;

    return 0;
}
