# Test if tf listner works correctly
using RobotOS
@rosimport std_msgs.msg: Header, Bool
@rosimport geometry_msgs.msg: Transform, TransformStamped, Vector3, Quaternion
rostypegen(@__MODULE__)
import .std_msgs.msg: Header, BoolMsg
import .geometry_msgs.msg: Transform, TransformStamped, Vector3, Quaternion

using ROSTransforms.tf2_ros
using Test

# Initialize node
init_node("listner", anonymous=true) # delete

# Setup the transform listener
buffer = Buffer()
sleep(10)

# Wait and collect the transform
transform = lookupTransform(buffer, "child_link", "parent_link", RobotOS.now(),
                            Duration(10))
display()
display(transform)
display()
@testset "Buffer" begin
    tf_correct = TransformStamped(Header(0, RobotOS.now(), "parent_link"),
                                  "child_link",
                                  Transform(Vector3(1., 2., 3.),
                                            Quaternion(0, 0, 0, 1.))
                                  )
    @test transform == tf_correct
end

# Publsih success
pub = Publisher{BoolMsg}("/success", queue_size=10)
for i ∈ 1:100
    publish(pub, BoolMsg(true))
    rossleep(.1)
end
