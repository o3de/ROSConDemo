/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#pragma once

#include "AppleEvent.h"
#include <AzCore/EBus/EBus.h>
#include <AzCore/Component/EntityId.h>

namespace AppleKraken
{
   using Tags = AZStd::vector<AZStd::string>;
   using ApplesGatheredByTag = AZStd::unordered_map<AZStd::string, uint32_t>;

   //! Interface to notify about statistical events
   class DemoStatisticsNotifications : public AZ::EBusTraits
   {
   public:
       static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
       virtual ~DemoStatisticsNotifications() = default;
       virtual void AddApple(const AppleEvent& appleEvent) = 0;
       virtual void OnApplePickerSpawned(const AZ::EntityId& entityId) = 0;
       virtual void SetApplePickerStatus(const AZ::EntityId& entityId, const AZStd::string& status) = 0;
   };

   using DemoStatisticsNotificationBus = AZ::EBus<DemoStatisticsNotifications>;
} // namespace AppleKraken
