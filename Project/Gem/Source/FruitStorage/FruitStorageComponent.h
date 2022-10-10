/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "FruitStorageBus.h"
#include <AzCore/Component/Component.h>
#include <AzFramework/AzFrameworkModule.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

namespace AppleKraken
{
    //! Component responsible for storing counters of apples gathered by Kraken.
    class FruitStorageComponent
        : public AZ::Component
        , public FruitStorageRequestsBus::Handler
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(FruitStorageComponent, "{9AC0B456-9C29-4EDD-AD25-6FAA57D253C5}", AZ::Component, FruitStorageRequestsBus::Handler);

        // AZ::Component interface implementation.
        FruitStorageComponent() = default;
        ~FruitStorageComponent() = default;
        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

        ApplesGatheredByTag GetTotalGatheredAppleCount(const Tags& tags = {}) const override;
        uint32_t GetCurrentStorageAppleCount() const override;
        void AddApple(const Tags& tags = {}) override;

    private:
        void SpawnCrate();
        void PreSpawn(AzFramework::EntitySpawnTicket::Id, AzFramework::SpawnableEntityContainerView);
        void displayNumberOfApples(int num);
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        AZ::Data::Asset<AzFramework::Spawnable> m_crateSpawnable;
        AzFramework::EntitySpawnTicket m_crateTicket;
        AZ::EntityId m_crateDropPoint;
        uint32_t m_crateCapacity = 0;
        uint32_t m_applesGathered = 0;
        uint32_t m_applesInStorage = 0;
        ApplesGatheredByTag m_tagsStored;
        AZ::EntityId m_ui_entity;
        const static AZStd::string kAppleGatheredElementName;
    };
} // namespace AppleKraken
