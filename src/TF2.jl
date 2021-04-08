module TF2

using RobotOS
@rosimport geometry_msgs.msg: TransformStamped
rostypegen(@__MODULE__)
import .geometry_msgs.msg: TransformStamped

using PyCall
import Base.==

const __tf__ = PyCall.PyNULL()

function __init__()
    copy!(__tf__, pyimport("tf"))
end

export TransformBroadcaster, sendTransform, TransformListener, lookupTransform, waitForTransform

"""
    Extending the equality operator (==) functionality to the TransformStamped
type. Useful for testing.
"""
function Base.:(==)(tf1::TransformStamped, tf2::TransformStamped)
    transform1 = tf1.transform; transform2 = tf1.transform

    if all(transform1.translation.x == transform2.translation.x,
           transform1.translation.y == transform2.translation.y,
           transform1.translation.z == transform2.translation.z,
           transform1.rotation.w == transform2.rotation.w,
           transform1.rotation.x == transform2.rotation.x,
           transform1.rotation.y == transform2.rotation.y,
           transform1.rotation.z == transform2.rotation.z)
        return true
    else
        return false
    end


end

"""
    TransformBroadcaster()
Create a transform broadcaster object.
"""
struct TransformBroadcaster
    o::PyObject
    function TransformBroadcaster(queue_size=100)
        new(__tf__.TransformBroadcaster(queue_size=queue_size))
    end
end

"""
    sendTransform(tf_broadcaster_obj, transform, time, child_frame, parent_frame)
Broadcast the transformation from tf frame child to parent on ROS topic "/tf".
"""
function sendTransform(tb::TransformBroadcaster,
                       translation::Vector,
                       rotation::Vector,
                       time::Time,
                       child_frame_id::AbstractString,
                       parent_frame_id::AbstractString)
    trans = translation
    rot = rotation
    pytime = convert(PyObject, pytime)
    pycall(tb.o.sendTransform, PyAny, trans, rot, pytime, child_frame, parent_frame)
end

"""
    sendTransform(tf_broadcaster, transform)
Broadcast the transformation from tf frame child to parent on ROS topic "/tf".
"""
function sendTransformMessage(tb::TransformBroadcaster,
                              tranStamped::TransformStamped)
    trans = transform.trans
    rot = transform.rot
    time = TransformStamped.header.stamp
    child_frame_id = tranStamped.child_frame_id
    parent_frame_id = tranStamped.header.frame_id

    sendTransform(tb, trans, rot, time, child_frame_id, parent_frame_id)
end

"""
    TransformListener()
Create a transform listener object.
"""
struct TransformListener
    o::PyObject
    function TransformListener(args...; kwargs...)
        new(__tf__.TransformListener(args...; kwargs...))
    end
end

"""
    generate_error_message(err)
Retrun error message string which includes both exception type and error massage information.
"""
function generate_error_message(err)
    exception_type = err.T.__name__
    error_message = exception_type * ": $(err.val.args[1])"
end

"""
    lookupTransform(tf_listener_obj, target, source, time)
Return tuple of (position, quaternion).
"""
function lookupTransform(tl::TransformListener,
                         target_frame::AbstractString,
                         source_frame::AbstractString,
                         time::Time)
    pytime = convert(PyObject, time)
    try
        trans, rot = pycall(tl.o.lookupTransform, PyAny, target_frame, source_frame, time)
        return Transform(trans, rot)
    catch err
        if isa(err, PyCall.PyError)
            error_massage = generate_error_message(err)
            error(error_massage)
        else
            rethrow(err)
        end
    end
end

"""
    waitForTransform(tf_listener_obj, target, source, time, timeout, pypolling_sleep_duration)
Waits for the given transformation to become available. If the timeout occurs before the transformation becomes available, raises an exception.
"""
function waitForTransform(tl::TransformListener,
                          target_frame::AbstractString,
                          source_frame::AbstractString,
                          time::Time,
                          timeout::Duration)
    pytime = convert(PyObject, time)
    pytimeout = convert(PyObject, timeout)
    polling_sleep_duration = convert(PyObject, pypolling_sleep_duration)
    try
        pycall(tl.o.waitForTransform, PyAny, target_frame, source_frame,
               pytime, pytimeout)
    catch err
        if isa(err, PyCall.PyError)
            error_message = generate_error_message(err)
            error(error_message)
        else
            rethrow(err)
        end
    end
end




end
