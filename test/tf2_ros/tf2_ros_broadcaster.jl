# Test if tf broadcaster works correctly
using RobotOS
@rosimport std_msgs.msg: Header, Bool
@rosimport geometry_msgs.msg: Transform, TransformStamped, Vector3, Quaternion
rostypegen(@__MODULE__)
import .std_msgs.msg: Header, BoolMsg
import .geometry_msgs.msg: Transform, TransformStamped, Vector3, Quaternion


using ROSTransforms.tf2_ros
using Test


init_node("broadcaster", anonymous=true) # delete
tb = TransformBroadcaster()

# Build the success callback function
success = false
function cb(msg::BoolMsg)
    global success = msg.data
end
sub1 = Subscriber{BoolMsg}("/success", cb, queue_size=10)

r = Rate(10.0)
for i ∈ 1:10000
    tf_correct = TransformStamped(Header(0, RobotOS.now(), "parent_link"),
                                  "child_link",
                                  Transform(Vector3(1., 2., 3.),
                                            Quaternion(0, 0, 0, 1.))
                                  )

    sendTransform(tb, tf_correct)

    success && break  # Break if the listener heard the broadcaster
    rossleep(r)
end

@testset "TransformBroadcaster" begin
    @test success  # Success if listener heard
end
