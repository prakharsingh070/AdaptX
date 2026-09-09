# ADAPT-X Constraints

Do not:

- fabricate benchmark results
- hardcode fake sensor values
- claim real-world autonomous-driving safety
- replace algorithms without documenting the change
- introduce unnecessary frameworks
- tightly couple frontend and perception logic
- put business logic inside UI components
- create giant monolithic files
- duplicate data models
- randomly change API contracts
- remove tests to make builds pass
- hide errors
- silently fall back to fake data

If real sensor or CARLA data is unavailable, clearly label the system as simulation, demo, replay, or unavailable mode. Preserve the distinction in API responses, logs, and dashboard views.
