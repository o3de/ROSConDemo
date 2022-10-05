/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/RTTI/BehaviorContext.h>

namespace AppleKraken
{
    using Tags = AZStd::vector<AZStd::string>;
    using ApplesGatheredByTag = AZStd::unordered_map<AZStd::string, uint32_t>;

    //! Interface handing fruits storage requests
    class FruitStorageRequests : public AZ::ComponentBus
    {
    public:
        AZ_RTTI(SpawnerRequests, "{129EAABF-706E-4DC1-B272-93EEACE3E893}");
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;

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
