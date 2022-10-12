/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FruitStorageComponent.h"
#include "FruitStorageBus.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzToolsFramework/UI/PropertyEditor/PropertyEditorAPI.h>
#include <LyShine/Bus/UiCanvasBus.h>
#include <LyShine/Bus/UiTextBus.h>
#include <LyShine/Bus/World/UiCanvasRefBus.h>

namespace AppleKraken
{

    FruitStorageComponent::FruitStorageComponent()
    {

        m_postSimulateHandler = AzPhysics::SystemEvents::OnPostsimulateEvent::Handler(
        [this](float deltaTime)
        {
            AZ_Printf("FruitStorageComponent", "FruitStorageComponentClear\n");
            AppleKraken::FruitStorageComponent::ResetCounter();
            m_postSimulateHandler.Disconnect();
        });
    }

    void FruitStorageComponent::Activate()
    {
        FruitStorageRequestsBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
        auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        if (physicsSystem)
        {
            physicsSystem->RegisterPostSimulateEvent(m_postSimulateHandler);
        }

    }

    void FruitStorageComponent::Deactivate()
    {

        FruitStorageRequestsBus::Handler::BusDisconnect(GetEntityId());
    }

    void FruitStorageComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<FruitStorageComponent, AZ::Component>()
                ->Version(2)
                ->Field("Crate", &FruitStorageComponent::m_crateSpawnable)
                ->Field("Capacity", &FruitStorageComponent::m_crateCapacity)
                ->Field("CrateDropPoint", &FruitStorageComponent::m_crateDropPoint)
                ->Field("UiEntity", &FruitStorageComponent::m_ui_entity);


            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<FruitStorageComponent>("Fruit Storage", "Fruit storage component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "Manages Kraken capacity and spawns a crate when full")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "AppleKraken")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &FruitStorageComponent::m_crateSpawnable, "Crate", "Crate spawnable")
                    ->DataElement(AZ::Edit::UIHandlers::EntityId, &FruitStorageComponent::m_crateCapacity, "Capacity", "Capacity")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &FruitStorageComponent::m_crateDropPoint,
                        "Crate drop point",
                        "Place this entity behind the robot")
                    ->DataElement(
                            AZ::Edit::UIHandlers::EntityId,
                            &FruitStorageComponent::m_ui_entity,
                            "Entity with Ui canvas reference ",
                            "Entity with UI canvas, that contains text element named ");

            }
        }
    }

    void FruitStorageComponent::SpawnCrate()
    {
        if (!m_crateTicket.IsValid())
        {
            m_crateTicket = AzFramework::EntitySpawnTicket(m_crateSpawnable);
        }

        auto spawner = AZ::Interface<AzFramework::SpawnableEntitiesDefinition>::Get();
        AzFramework::SpawnAllEntitiesOptionalArgs optionalArgs;
        optionalArgs.m_preInsertionCallback = [this](auto id, auto view)
        {
            this->PreSpawn(id, view);
        };
        AZ_TracePrintf("FruitStorageComponent", "Spawning a crate\n");
        spawner->SpawnAllEntities(m_crateTicket, optionalArgs);
    }

    void FruitStorageComponent::PreSpawn(
        AzFramework::EntitySpawnTicket::Id id [[maybe_unused]], AzFramework::SpawnableEntityContainerView view)
    {
        if (view.empty())
        {
            AZ_Warning("FruitStorageComponent", false, "Can not spawn a crate - no entities\n");
            return;
        }

        AZ::Entity* dropPointEntity = nullptr;
        if (m_crateDropPoint.IsValid())
        {
            AZ::ComponentApplicationBus::BroadcastResult(dropPointEntity, &AZ::ComponentApplicationRequests::FindEntity, m_crateDropPoint);
        }
        if (dropPointEntity == nullptr)
        {
            AZ_Warning("FruitStorageComponent", false, "Crate drop point not set or invalid, spawning a crate within this entity\n");
            dropPointEntity = GetEntity();
        }

        AZ::Entity* root = *view.begin();
        auto* crateTransformInterface = root->FindComponent<AzFramework::TransformComponent>();
        auto entityTransform = dropPointEntity->FindComponent<AzFramework::TransformComponent>();
        crateTransformInterface->SetWorldTM(entityTransform->GetWorldTM());
    }

    ApplesGatheredByTag FruitStorageComponent::GetTotalGatheredAppleCount(const Tags& tags) const
    {
        ApplesGatheredByTag result;
        if (tags.empty())
        {
            result["all"] = m_applesGathered;
        }
        else
        {
            for (const auto& tag : tags)
            {
                if (m_tagsStored.contains(tag))
                {
                    result[tag] = m_tagsStored.at(tag);
                }
            }
        }
        return result;
    }

    uint32_t FruitStorageComponent::GetCurrentStorageAppleCount() const
    {
        return m_applesInStorage;
    }

    void FruitStorageComponent::AddApple(const Tags& tags)
    {
        m_applesGathered++;
        m_applesInStorage++;
        for (const auto& tag : tags)
        {
            m_tagsStored.contains(tag) ? m_tagsStored[tag]++ : m_tagsStored[tag] = 1;
        }

        if (m_applesInStorage >= m_crateCapacity)
        {
            SpawnCrate();
            m_applesInStorage = 0;
        }

        m_allApplesGathered++;
        displayNumberOfApples(m_allApplesGathered);
    }

    void FruitStorageComponent::displayNumberOfApples(int num){
        AZ_Printf("FruitStorageComponent",  "displayNumberOfApples %d \n", num);
        AZ::EntityId ui_canvas;

        EBUS_EVENT_ID_RESULT(ui_canvas, m_ui_entity, UiCanvasRefBus, GetCanvas);
        if (!ui_canvas.IsValid())
        {
            AZ_Warning("FruitStorageComponent", false, "%s doest not have UiCanvasRef\n", m_ui_entity.ToString().c_str());
            return;
        }

        AZ::EntityId ui_text_entity;
        UiCanvasBus::EventResult(ui_text_entity,
                                 ui_canvas,
                                 &UiCanvasInterface::FindElementEntityIdByName,
                                 kAppleGatheredElementName);
        if (!ui_text_entity.IsValid())
        {
            AZ_Warning("FruitStorageComponent", false,  "ui canvas doest not have %s \n", kAppleGatheredElementName.c_str());
            return;
        }
        AZStd::string t = "Apples gathered : " + AZStd::to_string(num);
        UiTextBus::Event(ui_text_entity, &UiTextInterface::SetText, t);
    }

    void FruitStorageComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        displayNumberOfApples(m_allApplesGathered);
        AZ::TickBus::Handler::BusDisconnect();
    }

    void FruitStorageComponent::ResetCounter(){
        FruitStorageComponent::m_allApplesGathered.store(0);
    }

    const AZStd::string FruitStorageComponent::kAppleGatheredElementName{"AppleGatheredElement"};
    AZStd::atomic_int32_t FruitStorageComponent::m_allApplesGathered {0};

} // namespace AppleKraken
