/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/RTTI/BehaviorContext.h>

namespace AppleKraken
{
    //! Notifications related to apple picking
    class ApplePickingNotifications : public AZ::EBusTraits
    {
    public:
        virtual void ApplePicked(AZ::EntityId appleId) = 0;
        virtual void AppleRetrieved() = 0;
        virtual void PickingFailed(AZ::EntityId appleId, const AZStd::string& reason) = 0;
    };

    using ApplePickingNotificationBus = AZ::EBus<ApplePickingNotifications>;
} // namespace AppleKraken