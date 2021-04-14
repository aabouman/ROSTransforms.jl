module tf2_ros

    using RobotOS
    @rosimport geometry_msgs.msg: TransformStamped
    rostypegen(@__MODULE__)
    import .geometry_msgs.msg: TransformStamped

    using PyCall
    import Base.==

    const __tf__ = PyCall.PyNULL()
    function __init__()
        copy!(__tf__, pyimport("tf2_ros"))
    end

    export Buffer, TransformBroadcaster
    export sendTransform, lookupTransform, waitForTransform

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


# ============================================================================#
#               tf2_ros TransformBroadcaster object and functions
# ============================================================================#
    """
        TransformBroadcaster()
    Create a transform broadcaster object.
    """
    struct TransformBroadcaster
        o::PyObject
        function TransformBroadcaster()
            new(__tf__.TransformBroadcaster())
        end
    end

    """
        sendTransform(tf_broadcaster_obj, transform, time, child_frame, parent_frame)
    Broadcast the transformation from tf frame child to parent on ROS topic "/tf".
    """
    function sendTransform(tb::TransformBroadcaster,
                           transform::Any)
        pycall(tb.o.sendTransform, PyAny, transform)
    end

    """
    Publish set of transforms
    """
    function sendTransform(tb::TransformBroadcaster,
                           transforms::Vector{Any})
        pycall(tb.o.sendTransform, PyAny, transforms)
    end


    # ============================================================================#
    #                       tf2_ros Buffer object and functions
    # ============================================================================#
    """
        Buffer(; cache_time=Duration(1000))
    Create a Buffer object, which holds on to transforms for time cache_time
    """
    struct Buffer
        o::PyObject
        function Buffer(; cache_time=Duration(1000))
            pyCache_time = convert(PyObject, cache_time)
            new(__tf__.Buffer(cache_time=pyCache_time))
        end
    end

    """
        generate_error_message(err)
    Return error message string which includes both exception type and error massage information.
    """
    function generate_error_message(err)
        exception_type = err.T.__name__
        error_message = exception_type * ": $(err.val.args[1])"
    end

    """
        lookupTransform(tf_listener_obj, target, source, time)
    Return tuple of (position, quaternion).
    """
    function lookupTransform(tf_buffer::Buffer,
                             target_frame::AbstractString,
                             source_frame::AbstractString,
                             time::Time,
                             timeout::Duration)
        pytime = convert(PyObject, time)
        pytimeout = convert(PyObject, timeout)
        try
            transStamp = pycall(tf_buffer.o.lookup_transform, PyAny,
                                target_frame, source_frame, pytime, pytimeout)
            return transStamp
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
