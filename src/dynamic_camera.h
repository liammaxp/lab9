#ifndef DYNAMIC_CAMERA_H
#define DYNAMIC_CAMERA_H

#include <iostream>
#include <limits>

#include "camera.h"

constexpr float INF = std::numeric_limits<float>::infinity();

// AABB in 1D
struct Interval {
    float min;
    float max;

    constexpr bool contains(const Interval& other) const {
        return (min <= other.min) && (max >= other.max);
    }

    constexpr bool contains(float x) const { return (min <= x) && (x <= max); }

    // signedDistance returns a positive smallest distance of x from the interval boundary
    // if it is not contained, else it returns a smallest (in magnitude) negative distance
    // from the interval boundary.
    constexpr float signedDistance(float x) const {
        if (contains(x)) {
            // 0 or negative
            return std::max(x - max, min - x);
        }
        // positive
        return std::max(x - max, min - x);
    }
};

struct DynamicCamera {
    void update(Camera& camera,                           // the camera to update
                const sf::Vector2f& trackedPosWorld,      // world coords of player to track
                const sf::Vector2f& trackedVelocityWorld  // world velocity of player to track
    ) {
        // TODO: (HINTS/outline)
        // collect statistics on the player position and velocity in (normalized, if you like)
        // window coordinates. Compute lookAhead distance in world coords, adjusting for
        // if the player is moving left or right.
        // Now update lookAhead state using an EMA over its previous state and the lookAhead world
        // vector. Then determine if the player is in the dead zone, soft rejection zone, or hard
        // rejection zone. For each, perform EMA on the delta of the camera displacement (ignoring
        // lookahead) appropriately. Next,
        // set the camera center by offsetting the cameraUnadjustedCenter, combining lookAhead.
        // Finally update cameraUnadjustedCenter, but without lookahead.
        const float playerX = 0.5f + camera.worldToWindowPercentDist(
        trackedPosWorld - cameraUnadjustedCenter).x;
        float targetLookahead = 0.f;
        if (trackedVelocityWorld.x > 0.f) {
            targetLookahead = lookAhead;
        } 
        else if (trackedVelocityWorld.x < 0.f) {
            targetLookahead = -lookAhead;
        }
        const float targetLookaheadWorld =camera.windowPercentToWorldDist({targetLookahead, 0.f}).x;
        lookaheadState += alpha * (targetLookaheadWorld - lookaheadState);
        float deltaPercent = 0.f;
        float damping = 1.f;
        if (playerX < softZoneX.min) {
            deltaPercent = playerX - softZoneX.min;
        } 
        else if (playerX > softZoneX.max) {
            deltaPercent = playerX - softZoneX.max;
        } 
        else if (playerX < deadZoneX.min) {
            deltaPercent = playerX - deadZoneX.min;
            damping = alpha;
        } 
        else if (playerX > deadZoneX.max) {
            deltaPercent = playerX - deadZoneX.max;
            damping = alpha;
        }
        const float deltaWorld =camera.windowPercentToWorldDist({deltaPercent, 0.f}).x * damping;
        const sf::Vector2f newUnadjustedCenter = cameraUnadjustedCenter + sf::Vector2f{deltaWorld, 0.f};
        camera.setCameraCenter( newUnadjustedCenter + sf::Vector2f{lookaheadState, 0.f});
        cameraUnadjustedCenter = newUnadjustedCenter;
    }

    float alpha;  // alpha to use for damping of camera position, *as well as* for smoothing of
                  // lookahead offset.
    Interval deadZoneX;  // percentage interval along x-axis to use for dead zone in [0, 1]
    // percentage interval along x-axis to use for soft rejection zone in [0, 1]
    Interval softZoneX = {-INF, INF};
    // (everything beyond softZone is considered hard zone).
    float lookAhead;  // screen-percentage lookahead
    mutable sf::Vector2f
        cameraUnadjustedCenter;      // the camera world center from last frame, ignoring lookahead
    mutable float lookaheadState{};  // the EMA of the accumulated lookahead offset in world coords
};

#endif // DYNAMIC_CAMERA_H
