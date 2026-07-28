#include "playertia/engine.h"

#include <assert.h>
#include <string.h>

int main(void) {
  playertia_engine* engine = NULL;
  assert(playertia_engine_get_abi_version() == PLAYERTIA_ABI_VERSION);
  assert(playertia_engine_get_state_size() > 0U);
  assert(playertia_engine_create(PLAYERTIA_ABI_VERSION, &engine) == PLAYERTIA_OK);
  assert(engine != NULL);

  {
    const double angles[PLAYERTIA_HAND_COUNT] = {0.0, 1.0, 2.0};
    const float velocities[PLAYERTIA_HAND_COUNT] = {0.0F, 0.0F, 0.0F};
    const playertia_vec3 omega = {0.0F, 0.0F, 0.0F};
    assert(playertia_engine_reset(engine, angles, velocities, omega) == PLAYERTIA_OK);
  }

  {
    playertia_snapshot snapshot;
    memset(&snapshot, 0, sizeof(snapshot));
    assert(playertia_engine_get_snapshot(
               engine, &snapshot, (uint32_t)sizeof(snapshot)) == PLAYERTIA_OK);
    assert(snapshot.abi_version == PLAYERTIA_ABI_VERSION);
    assert(snapshot.struct_size == sizeof(snapshot));
  }

  playertia_engine_destroy(engine);
  return 0;
}
