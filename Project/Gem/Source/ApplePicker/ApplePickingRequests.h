/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

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
        //! @param globalAppleBoundingBox global bounding box for the apple.
        //! @param appleId
        //! Note this could be changed to local Aabb for real use cases.
        //! Global can be queried once, local is less resilient to robot instability, mini movement.
        //! Note that this can get a while and result will be signalled through ApplePickingNotifications.
        virtual void PickApple(AZ::Aabb globalAppleBoundingBox, AZ::EntityId appleId) = 0;

        //! Request to store currently held apple and retrieve manipulator into a travel position.
        virtual void FinishPicking() = 0;

        //! Return area covered by effector.
        //! @returns an object bounding box which is a region where apples can be picked.
        virtual AZ::Obb GetEffectorReachArea() = 0;
    };

    class ApplePickingBusTraits : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
    };
    using ApplePickingRequestBus = AZ::EBus<ApplePickingRequests, ApplePickingBusTraits>;
    using ApplePickingInterface = AZ::Interface<ApplePickingRequests>;
} // namespace AppleKraken
