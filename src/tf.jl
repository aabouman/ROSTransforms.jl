module tf

    using RobotOS
    using PyCall
    using Rotations
    using StaticArrays
    using LinearAlgebra: inv
    import Base.==
    import Base.*
    import Base.rand
    import Base.inv

    const __tf__ = PyCall.PyNULL()

    function __init__()
        copy!(__tf__, pyimport("tf"))
    end

    export Transform
    export rand
    export TransformBroadcaster, sendTransform
    export TransformListener, lookupTransform, waitForTransform
    export transformWrench, poseToTransform, poseStampedToTransform

    """
       Transform(trans, rot)
    Create transform object.
    """
    struct Transform
        trans::Array{Float64}
        rot::Array{Float64}
    end

# =========================================================================== #
#                      Useful functions on Transform type
# =========================================================================== #
    function Base.:(==)(tf1::Transform, tf2::Transform)
        (all(tf1.trans .== tf2.trans) && all(tf1.rot .== tf2.rot))
    end

    function Base.:(*)(tf1::Transform, tf2::Transform)::Transform
        # Apply transform
        tf3Mat = twist2RBT(tf1) * twist2RBT(tf2)
        tf3 = RBT2twist(tf3Mat)
        return tf3
    end

    function Base.inv(tf::Transform)::Transform
        # Apply transform
        tf_inv = RBT2twist(inv(twist2RBT(tf)))
        return tf_inv
    end

    function twist2RBT(tf::Transform)::Matrix
        x, y, z, w = tf.rot
        rotMat = Matrix(UnitQuaternion(w, x, y, z))

        tfMat = [rotMat   tf.trans;
                 [0 0 0]  1];
        return tfMat
    end

    function RBT2twist(tfMat::Matrix)::Transform
        # Translations
        trans = tfMat[1:3, 4]
        # Rotations
        rotMat = RotMatrix(SMatrix{3,3}(tfMat[1:3, 1:3]))
        quat = UnitQuaternion(rotMat)
        rot = [quat.x, quat.y, quat.z, quat.w]

        tf = Transform(trans, rot)
        return tf
    end

    function Base.rand(::Type{Transform})::Transform
        trans = rand(3)
        quat = rand(UnitQuaternion)
        rot = [quat.x, quat.y, quat.z, quat.w]
        return Transform(trans, rot)
    end

# ============================================================================#
#                 tf TransformBroadcaster object and functions
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
        sendTransformMessage(tb, translation, rotation, time, child, parent)
    Broadcast the transformation from tf frame child to parent on ROS topic "/tf".
    """
    function sendTransform(tb::TransformBroadcaster,
                           translation::Vector,
                           rotation::Vector,
                           time::Time,
                           child::AbstractString,
                           parent::AbstractString)
        trans = translation
        rot = rotation
        pytime = convert(PyObject, time)
        pycall(tb.o.sendTransform, PyAny, trans, rot, pytime, child, parent)
    end

    """
        sendTransformMessage(tb::TransformBroadcaster, transform)
    Broadcast the transformation from tf frame child to parent on ROS topic "/tf".
    """
    function sendTransformMessage(tb::TransformBroadcaster,
                                  transform::Any)
        pytransform = convert(PyObject, transform)
        pycall(tb.o.sendTransformMessage, PyAny, pytransform)
    end

# ============================================================================#
#                 tf TransformListener object and functions
# ============================================================================#
    """
        TransformListener()
    Create a transform listener object.
    """
    struct TransformListener
        o::PyObject
        function TransformListener()
            new(__tf__.TransformListener())
        end
    end

    """
        generate_error_message(err)
    Retrun error message string which includes both exception type and error massage information.
    """
    function generate_error_message(err)
        exception_type = err.T.__name__
        error_massage = exception_type * ": $(err.val.args[1])"
    end

    """
        lookupTransform(tf_listener_obj, target, source, time)
    Return tuple of (position, quaternion).
    """
    function lookupTransform(tl::TransformListener,
                             target_frame::AbstractString,
                             source_frame::AbstractString,
                             pytime::Time)
        time = convert(PyObject, pytime)
        try
            trans, rot = pycall(tl.o.lookupTransform, PyAny, target_frame,
                                source_frame, time)
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
                              pytime::Time,
                              pytimeout::Duration;
                              pypolling_sleep_duration = Duration(0.01))
        time = convert(PyObject, pytime)
        timeout = convert(PyObject, pytimeout)
        polling_sleep_duration = convert(PyObject, pypolling_sleep_duration)
        try
            pycall(tl.o.waitForTransform, PyAny, target_frame, source_frame,
                   time, timeout, polling_sleep_duration)
        catch err
            if isa(err, PyCall.PyError)
                error_massage = generate_error_message(err)
                error(error_massage)
            else
                rethrow(err)
            end
        end
    end

# ============================================================================#
#                           Helpful functions
# ============================================================================#
    """
    """
    function toSkew(vect::Vector)
        all(size(vect) .== (3,)) || error("vect must be 3 vector")
        return [0       -vect[3]  vect[2];
                vect[3]  0       -vect[1];
               -vect[2]  vect[1]  0];
    end

    """
    """
    function transformWrench(transform::Transform,
                             wrench::Any,
                             )
        x, y, z, w = transform.rot
        𝑞ₛₑᵀ = inv(UnitQuaternion(w, x, y, z))
        p̂ₛₑ = toSkew(transform.trans)

        Adᵀ = [𝑞ₛₑᵀ   zeros(3, 3);
               -𝑞ₛₑᵀ*p̂ₛₑ  𝑞ₛₑᵀ]
        𝑓ₛ = [wrench.force.x, wrench.force.y, wrench.force.z,
             wrench.torque.x, wrench.torque.y, wrench.torque.z];
        𝑓ₑ = Adᵀ * 𝑓ₛ

        ret_wrench = deepcopy(wrench)
        ret_wrench.force.x = 𝑓ₑ[1]
        ret_wrench.force.y = 𝑓ₑ[2]
        ret_wrench.force.z = 𝑓ₑ[3]
        ret_wrench.torque.x = 𝑓ₑ[4]
        ret_wrench.torque.y = 𝑓ₑ[5]
        ret_wrench.torque.z = 𝑓ₑ[6]

        return ret_wrench
    end

    """
    """
    function poseToTransform(pose::Any)::Transform
        trans = [pose.position.x, pose.position.y, pose.position.z]
        rot = [pose.orientation.x, pose.orientation.y,
               pose.orientation.z, pose.orientation.w]

        return Transform(trans, rot)
    end

    """
    """
    function poseStampedToTransform(poseStamped::Any)::Transform
        return poseToTransform(poseStamped.pose)
    end

    """
    """
    function transformPose(transform::Transform,
                           pose::Any,
                           )
        x, y, z, w = transform.rot
        𝑞ₛₑᵀ = inv(UnitQuaternion(w, x, y, z))
        p̂ₛₑ = toSkew(transform.trans)
        Adᵀ = [𝑞ₛₑᵀ   zeros(3, 3);
               -𝑞ₛₑᵀ*p̂ₛₑ  𝑞ₛₑᵀ]
        𝑓ₛ = [wrench.force.x, wrench.force.y, wrench.force.z,
             wrench.torque.x, wrench.torque.y, wrench.torque.z];
        𝑓ₑ = Adᵀ * 𝑓ₛ
    end
end
