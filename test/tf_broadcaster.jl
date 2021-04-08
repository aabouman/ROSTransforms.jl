using RobotOS
@rosimport std_msgs.msg: Bool
rostypegen(@__MODULE__)
import .std_msgs.msg: BoolMsg

using ROSTransforms.TF
using Test


init_node("broadcaster", anonymous=true) # delete
tb = TransformBroadcaster()
tf_ref = Transform([1, 2, 3.], [0, 0, 0, 1.])

# Build the success callback function
success = false
function cb(msg::BoolMsg)
    global success = msg.data
end
sub1 = Subscriber{BoolMsg}("/success", cb, queue_size=10)

r = Rate(20.0)
for i ∈ 1:1000
    sendTransform(tb, tf_ref, RobotOS.now(), "child_link", "parent_link")

    success && break  # Break if the listener heard the broadcaster
    rossleep(r)
end

@testset "TransformBroadcaster" begin
    @test success  # Success if listener heard
end
