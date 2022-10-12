/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/Math/Aabb.h>

namespace AppleKraken
{
    //! Important states of the Kraken effector (manipulator with a vacuum nozzle)
    enum class EffectorState : int16_t
    {
        INVALID = -1, //!< Invalid state. Requires an additional context that could help user understand what happened. @see PickingState.
        IDLE = 0, //!< Idle state / position, suitable for robot moving around the environment.
        PREPARED = 10, //!< State and position which are ready for picking tasks.
        PICKING = 20, //!< The effector is on its way to pick fruit.
        RETRIEVING = 30 //!< The effector is retrieving a fruit to storage position.
    };

    //! A task to pick a single apple.
    struct PickAppleTask
    {
        bool IsValid() const
        {
            return m_appleEntityId.IsValid();
        }

        void Invalidate()
        {
            m_appleEntityId = AZ::EntityId();
        }

        AZ::EntityId m_appleEntityId; //!< EntityId of the apple. Can be Invalid if the information is not available (check IsValid()).
        AZ::Aabb m_appleBoundingBox; //!< Bounding box of the apple to pick.
        AZ::Vector3 m_middle; //!< Middle point of Apple
    };

    //! A structure holding a state of effector, including optional progress and descriptive information.
    struct PickingState
    {
        EffectorState m_effectorState = EffectorState::IDLE; //!< Current state of effector.
        PickAppleTask m_currentTask; //!< Only valid for EffectorState::PICKING and EffectorState::RETRIEVING
        float m_taskProgress = 0.0f; //!< Optional field signalling progress within current state (picking/retrieving).
        AZStd::string m_description; //!< Optional descriptive field to inform the user.
    };

    using StateTransition = AZStd::pair<EffectorState, EffectorState>;

    struct TransitionHash
    {
        size_t operator()(const AZStd::pair<EffectorState, EffectorState>& p) const
        {
            int16_t first = static_cast<int16_t>(p.first);
            int16_t second = static_cast<int16_t>(p.second);
            size_t combined = (size_t)first << 16 | second;
            return combined;
        }
    };
} // namespace AppleKraken
