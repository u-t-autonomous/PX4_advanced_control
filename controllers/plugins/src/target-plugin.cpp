#include <boost/bind.hpp>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/common/common.hh>
#include <ignition/transport/Node.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/transport/Node.hh"

namespace gazebo
{
    class ChangeTargetVisualPlugin : public VisualPlugin
    {
    public:
        void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf) override
        {
            gzmsg << "Loading target plugin";
            this->model = _parent; //Initialize parent model element
            // Subscribe to ContainPlugin output
            if (!_sdf-> HasElement("contains_topic"))
            {
                gzerr << "Missing required parameter <contains_topic>, "
                      << "plugin will not be initialized" << std::endl;
            }
            std::string topic(_sdf->Get<std::string>("contains_topic"));
            std::function<void(const ignition::msgs::Boolean &)> cb =
                [ = ](const ignition::msgs::Boolean & _msg)
            {
                ChangeTargetVisualPlugin::OnContainPluginMsg(_msg);
            };
            const bool containSub = this->node.Subscribe(topic, cb);
            if (!containSub)
            {
                gzerr << "Failed to subscribe to [" << topic << "]\n";
            }

        }

        void OnContainPluginMsg(const ignition::msgs::Boolean &_msg)
        {
            common::Color green(0, 1, 0);
            common::Color red(1, 0, 0);
            if (_msg.data())
            {
                this->model->SetDiffuse(green);
                this->model->SetEmissive(green);
            }
            else
            {
                this->model->SetDiffuse(red);
                this->model->SetEmissive(red);
            }
        }

    private:
        ignition::transport::Node node;
        rendering::VisualPtr model;
    };
    GZ_REGISTER_VISUAL_PLUGIN(ChangeTargetVisualPlugin);
}