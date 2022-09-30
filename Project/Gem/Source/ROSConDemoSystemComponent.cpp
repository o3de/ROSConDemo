
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <Atom/RPI.Public/FeatureProcessorFactory.h>

#include "ROSConDemoSystemComponent.h"
#include "SunShaftsFeatureProcessor.h"

namespace ROSConDemo
{
    void ROSConDemoSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        SunShaftsFeatureProcessor::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROSConDemoSystemComponent, AZ::Component>()
                ->Version(0)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROSConDemoSystemComponent>("ROSConDemo", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void ROSConDemoSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("ROSConDemoService"));
    }

    void ROSConDemoSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("ROSConDemoService"));
    }

    void ROSConDemoSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("AtomBridgeService"));
    }

    void ROSConDemoSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ROSConDemoSystemComponent::ROSConDemoSystemComponent()
    {
        if (ROSConDemoInterface::Get() == nullptr)
        {
            ROSConDemoInterface::Register(this);
        }
    }

    ROSConDemoSystemComponent::~ROSConDemoSystemComponent()
    {
        if (ROSConDemoInterface::Get() == this)
        {
            ROSConDemoInterface::Unregister(this);
        }
    }

    void ROSConDemoSystemComponent::Init()
    {
    }

    void ROSConDemoSystemComponent::Activate()
    {

        ROSConDemoRequestBus::Handler::BusConnect();

        AZ::RPI::FeatureProcessorFactory::Get()->RegisterFeatureProcessor<SunShaftsFeatureProcessor>();

        auto* passSystem = AZ::RPI::PassSystemInterface::Get();
        AZ_Assert(passSystem, "Cannot get the pass system.");

        // Setup handler for load pass templates mappings
        m_loadTemplatesHandler = AZ::RPI::PassSystemInterface::OnReadyLoadTemplatesEvent::Handler(
            [passSystem]()
            {
                const char* passTemplatesFile = "Passes/SunShaftsPassTemplates.azasset";
                passSystem->LoadPassTemplateMappings(passTemplatesFile);
            });
        //passSystem->AddPassCreator(AZ::Name("SunShaftsFullScreenPass"), &SunShaftsFullScreenPass::Create);
        passSystem->ConnectEvent(m_loadTemplatesHandler);
    }

    void ROSConDemoSystemComponent::Deactivate()
    {
        AZ::RPI::FeatureProcessorFactory::Get()->UnregisterFeatureProcessor<SunShaftsFeatureProcessor>();

        m_loadTemplatesHandler.Disconnect();

        ROSConDemoRequestBus::Handler::BusDisconnect();
    }
}
