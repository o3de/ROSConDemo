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
#include <AzCore/RTTI/BehaviorContext.h>

namespace AppleKraken
{
    //! Notifications related to apple picking.
    class ApplePickingNotifications : public AZ::EBusTraits
    {
    public:
        //! The effector is ready for picking
        virtual void EffectorReadyForPicking() = 0;

        //! An apple was successfully picked.
        virtual void ApplePicked() = 0;

        //! An apple was successfully retrieved to storage and can count as harvested.
        virtual void AppleRetrieved() = 0;

        //! Apple picking failed
        //! @param reason reason for failure, e.g. "out of reach", "apple not found", "approach obstructed", etc.
        virtual void PickingFailed(const AZStd::string& reason) = 0;
    };

    using ApplePickingNotificationBus = AZ::EBus<ApplePickingNotifications>;
} // namespace AppleKraken