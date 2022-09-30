/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "PickingStructs.h"
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Math/Aabb.h>
#include <AzCore/Math/Obb.h>

namespace AppleKraken
{
    //! Requests for apple picking effector (manipulator)
    class ApplePickingRequests
    {
    public:
        AZ_RTTI(ApplePickingRequests, "{E70BC163-4AE0-4660-9769-1C3C7C3493A6}");
        virtual ~ApplePickingRequests() = default;

        //! Request to prepare for incoming apple picking tasks. Could be empty if manipulator is always ready.
        virtual void PrepareForPicking() = 0;

        //! PickApple by its global bounding box.
        //! @param appleTask task structure specifying which apple to pick.
        //! This task should only be issued if effector state is PREPARED.
        //! Note that the task can get a while and result will be signalled through ApplePickingNotifications.
        //! Progress can be checked @see GetEffectorState.
        //! This function returns immediately.
        virtual void PickApple(const PickAppleTask& appleTask) = 0;

        //! Request to store currently held apple and retrieve manipulator into a travel position.
        //! This function returns immediately. The effector should respond by transitioning to an IDLE state.
        virtual void FinishPicking() = 0;

        //! Request current effector state.
        //! This request should be called to inform about whether a task can be issued and whether the robot could start moving.
        //! @returns current state of the effector.
        virtual PickingState GetEffectorState() = 0;

        //! Return area covered by effector.
        //! @returns a global object bounding box which is a region where apples can be picked.
        virtual AZ::Obb GetEffectorReachArea() = 0;
    };

    class ApplePickingBusTraits : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
    };
    using ApplePickingRequestBus = AZ::EBus<ApplePickingRequests, ApplePickingBusTraits>;
    using ApplePickingInterface = AZ::Interface<ApplePickingRequests>;
} // namespace AppleKraken
