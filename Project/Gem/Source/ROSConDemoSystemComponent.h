
#pragma once

#include <AzCore/Component/Component.h>
#include <Atom/RPI.Public/Pass/PassSystemInterface.h>

#include <ROSConDemo/ROSConDemoBus.h>

namespace ROSConDemo
{
    class ROSConDemoSystemComponent
        : public AZ::Component
        , protected ROSConDemoRequestBus::Handler
    {
    public:
        AZ_COMPONENT(ROSConDemoSystemComponent, "{194FFE4C-CA95-400E-BCA2-CB5083ABEC5F}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        ROSConDemoSystemComponent();
        ~ROSConDemoSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // ROSConDemoRequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////

    private:
        //! Used for loading the pass templates
        AZ::RPI::PassSystemInterface::OnReadyLoadTemplatesEvent::Handler m_loadTemplatesHandler;
    };
}
