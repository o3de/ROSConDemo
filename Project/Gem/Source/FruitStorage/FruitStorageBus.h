/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "../DemoStatistics/AppleEvent.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/EBus/Event.h>

namespace AppleKraken
{
    using ApplesGatheredByTag = AZStd::unordered_map<AZStd::string, uint32_t>;

    //! Interface handing fruits storage requests
    class FruitStorageRequests : public AZ::EBusTraits
    {
    public:
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        using BusIdType = AZ::EntityId;

        virtual ~FruitStorageRequests() = default;

        //! Gets counts of apples gathered with certain tags. When no tags are provided, returns number of all apples gathered.
        virtual ApplesGatheredByTag GetTotalGatheredAppleCount(const Tags& tags = {}) const = 0;

        //! Gets only apples currently in storage
        virtual uint32_t GetCurrentStorageAppleCount() const = 0;

        //! Adds one apple associated with certain tags to the gathered apples counters.
        virtual void AddApple(const Tags& tags = {}) = 0;
    };

    using FruitStorageRequestsBus = AZ::EBus<FruitStorageRequests>;
} // namespace AppleKraken
