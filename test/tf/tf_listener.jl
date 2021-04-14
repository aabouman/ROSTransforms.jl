# Test if tf listner works correctly
using RobotOS
@rosimport std_msgs.msg: Bool
rostypegen(@__MODULE__)
import .std_msgs.msg: BoolMsg

using ROSTransforms.tf
using Test

# Initialize node
init_node("listner", anonymous=true) # delete

# Setup the transform listener
tl = TransformListener()
# Wait and collect the transform
waitForTransform(tl, "parent_link", "child_link", Time(), Duration(1.0))
transform = lookupTransform(tl, "parent_link", "child_link", Time())


@testset "TransformListener" begin
    tf_correct = Transform([1, 2, 3.], [0, 0, 0, 1.])
    @test transform == tf_correct
end

# Publsih success
pub = Publisher{BoolMsg}("/success", queue_size=10)
for i ∈ 1:100
    publish(pub, BoolMsg(true))
    rossleep(.1)
end
