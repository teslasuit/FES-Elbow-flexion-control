from src.teslasuit_joint_control.suit_handler import Teslasuit
import easygui
from simple_pid import PID
import numpy as np
from dataclasses import dataclass
from scipy.spatial.transform import Rotation as R
from teslasuit_sdk.ts_mapper import TsBone2dIndex
import os
import sys
# ts_api_path = os.environ['TESLASUIT_PYTHON_API_PATH']
# sys.path.append(ts_api_path)


@dataclass
class BiomechanicalAngle():

    """
    Part of joint biomechanical system , containing properties of single biomechanical joint in single plane,
    including muscles that contol biomechanics at that plane and current and desired angles.

        Attributes
        ----------
        joint_name : str
            a name of the joint containing the angle
        plane : str ('saggital', 'frontal', 'transverse')
            biomechanical plane in which angle is defined
        agonist_muscle : str
            muscle group, contraction of which elicits positive angle changes in defined plane
        antagonist_muscle : str
            muscle group, contraction of which elicits negative angle changes in defined plane
        angle: int or None
            current biomechanical angle value
        desired_angle: int or None
            angle value that needs to be achieved using any control system

    """

    joint_name: str
    plane: str
    agonist_muscle: str
    antagonist_muscle: str
    angle = None
    desired_angle = 0


@dataclass
class Joint():

    """
    Part  of skeletal model, describing one single biomechanical joint containing one or several biomechanical angles(planes),
    including bones that forms the joits and their initial quaternions, obtained during calibration.

        Attributes
        ----------
        side : str ('Left', 'Right')
            side of the joint
        saggital_plane : BiomechanicalAngle or None
            biomechanical angle that describes movements in saggital plane
        frontal_plane : str
            biomechanical angle that describes movements in frontal plane
        transverse_plane : str
            biomechanical angle that describes movements in transverse plane
        proximal_bone: TsBone2dIndex
            Index of proximal bone that forms the joint (from TsBone2dIndex)
        distal_bone: int or None
            Index of proximal bone that forms the joint (from TsBone2dIndex)
        initial_proximal_bone_quat: List
            Calibration quaternion of proximal bone IMU
        initial_distal_bone_quat: List
            Calibration quaternion of distal bone IMU
    """

    side: str or None
    saggital_plane: BiomechanicalAngle or None
    frontal_plane: BiomechanicalAngle or None
    transverse_plane: BiomechanicalAngle or None
    proximal_bone: TsBone2dIndex
    distal_bone: TsBone2dIndex
    initial_proximal_bone_quat = None
    initial_distal_bone_quat = None
    initial_heading_rot_re = None


@dataclass
class SkeletalModel():
    """
    An object that defines skeletal model and contains all biomechanical joints
    """

    def __init__(self) -> None:

        self.left_elbow_joint = Joint(
            'Left',
            BiomechanicalAngle(
                'Elbow',
                'Saggital',
                "Biceps",
                "Triceps"),
            None,
            None,
            TsBone2dIndex.LeftUpperArm,
            TsBone2dIndex.LeftLowerArm)
        self.right_elbow_joint = Joint(
            'Right',
            BiomechanicalAngle(
                'Elbow',
                'Saggital',
                "Biceps",
                "Triceps"),
            None,
            None,
            TsBone2dIndex.RightUpperArm,
            TsBone2dIndex.RightLowerArm)


class SkeletalModelUpdater():

    """
    An updater of skeletal model biomechanics that uses Teslasuit MoCap output to calibrate and update joint angles.

        Attributes
        ----------
        suit : Teslasuit
            A suit object to get MoCap data from
        model : SkeletalModel
            A SkeletalModel obj. to update biomechanics in
        ini_saggital_vec: numpy.array
            A mask for the saggital plane quaternion transformations
        ini_frontal_vec: numpy.array
            A mask for the frontal plane quaternion transformations
        ini_transverse_vec: numpy.array
            A mask for the transverse plane quaternion transformations

        Methods
        -------
        calibrate()
            Gets an initial quats for all bones in model joints using Teslasuit T-pose calibration procedure
        update_angles()
            Calculates current angles for all planes in all joints of the models and updates their values
    """

    def __init__(self, suit: Teslasuit, model: SkeletalModel):
        """
        Parameters
        ----------
        suit : Teslasuit
            A suit object to get MoCap data from
        model : SkeletalModel
            A SkeletalModel obj. to update biomechanics in
        """

        self.suit = suit
        self.suit.start_mocap_streaming()
        self.ini_saggital_vec = np.array([0, 1, 0])
        self.ini_frontal_vec = np.array([1, 0, 0])
        self.ini_transverse_vec = np.array([0, 0, 1])
        self.model = model

    def calibrate(self):
        easygui.msgbox(
            "Get ready and straighten your extremities!.",
            'Mocap calibration',
            'OK')

        for joint in [self.model.__dict__[key]
                      for key in iter(self.model.__dict__)]:
            joint.initial_proximal_bone_quat = self.suit.get_q6(
                joint.proximal_bone)
            joint.initial_distal_bone_quat = self.suit.get_q6(
                joint.distal_bone)

            joint.initial_heading_rot = R.from_quat(
                joint.initial_proximal_bone_quat) * R.from_quat(joint.initial_distal_bone_quat).inv()

    def update_angles(self):

        for joint in [self.model.__dict__[key]
                      for key in iter(self.model.__dict__)]:
            q_proximal = self.suit.get_q6(joint.proximal_bone)
            q_distal = self.suit.get_q6(joint.distal_bone)

            if joint.saggital_plane is not None:

                vec1 = R.from_quat(q_proximal).apply(self.ini_saggital_vec)
                vec2 = (
                    joint.initial_heading_rot *
                    R.from_quat(q_distal)).apply(
                    self.ini_saggital_vec)
                joint.saggital_plane.angle = np.arccos(
                    vec1.dot(vec2)) * 180 / np.pi
                print(joint.saggital_plane.angle)

            if joint.frontal_plane is not None:

                vec1 = R.from_quat(q_proximal).apply(self.ini_frontal_vec)
                vec2 = (
                    joint.initial_heading_rot *
                    R.from_quat(q_distal)).apply(
                    self.ini_frontal_vec)
                joint.frontal_plane.angle = np.arccos(
                    vec1.dot(vec2)) * 180 / np.pi

            if joint.transverse_plane is not None:

                vec1 = R.from_quat(q_proximal).apply(self.ini_frontal_vec)
                vec2 = (
                    joint.initial_heading_rot *
                    R.from_quat(q_distal)).apply(
                    self.ini_transverse_vec)
                joint.transverse_plane.angle = np.arccos(
                    vec1.dot(vec2)) * 180 / np.pi


class SimplePID():
    """
    A class that creates PID controller.

        Attributes
        ----------
        pid : PID
            PID controller object
        parameters_list : List
            List of PID parameters (Proportional,Integral,Derivative)
        output: int
            Calculated stimulation output value

        Methods
        -------
        apply_pid(input)
            updates output value based on input error
        change_parameters()
            change controller parameters based on parameters_list
    """

    def __init__(self):

        self.parameters_list = [0, 0, 0]
        self.pid = PID(
            Kp=self.parameters_list[0],
            Ki=self.parameters_list[1],
            Kd=self.parameters_list[2],
            setpoint=0,
            sample_time=0.01,
            output_limits=(
                0,
                100))
        self.output = 0

    def apply_pid(self, input):
        """
         Parameters
        ----------
        input : float
            angular error value between current and desired biomechanical angles
        """

        self.output = int(self.pid(input))

    def change_parameters(self):
        self.pid.tunings = (
            self.parameters_list[0],
            self.parameters_list[1],
            self.parameters_list[2])

    def reset_pid(self):
        self.pid = PID(
            Kp=self.parameters_list[0],
            Ki=self.parameters_list[1],
            Kd=self.parameters_list[2],
            setpoint=0,
            sample_time=0.01,
            output_limits=(
                0,
                100))
    


class PIDMuscleControler(SimplePID):
    """
    A class that creates PID-based muscle stimulation controller.

        Attributes
        ----------
        suit : Teslasuit
            A suit object to send haptic output to
        haptic_channel : c_int or List
            Index(es) of corresponding haptic channels according to mapping
        s: str
            muscle location side
        m: str
            muscle location name
        pid : PID
            PID controller object
        parameters_list : List
            List of PID parameters (Proportional,Integral,Derivative)
        output: int
            Calculated stimulation output value
        Methods
        -------
        update_haptic_output (angular_error):
            updates stimulation output value based on input angular error
        send_haptic_output():
            sending and playing stimulation with current output values
            on corresponding to the controlled muscle channel.
    """

    def __init__(self, suit: Teslasuit, side_name, muscle_name):
        super(PIDMuscleControler, self).__init__()
        self.suit = suit
        self.haptic_channel = suit.ems_channels[side_name][muscle_name]
        self.s = side_name
        self.m = muscle_name

    def update_haptic_output(self, angular_error):
        """
        Parameters
        ----------
        angular_error : float
            angular error value between current and desired biomechanical angles
        """

        self.apply_pid(angular_error)

    def send_haptic_output(self):
        self.suit.haptic_play_touch(self.haptic_channel,
            pw=self.output,
            duration=100)
        print(
            'Haptic on {} {} with pw {} sent'.format(
                self.s,
                self.m,
                self.output))


class PIDPlaneControler():
    """
    A class that creates Plane controller based on agonist and antagonist muscles controllers.

        Attributes
        ----------
        agonist_controller : PIDMuscleControler
            Agonist muscle PID controller obj.
        antagonist_controller : PIDMuscleControler
            Antagonist muscle PID conroller obl.
        biomechanical_angle: BiomechanicalAngle
            Controlled BiomechanicalAngle obj.

        Methods
        -------
        update_haptic_output (angular_error):
            Calculates current angular error and updates stimulation output
            on the both agonist and antagonists controllers.
        send_haptic_output():
            Sending and playing stimulation with current output values
            on the corresponding to the controlled plane muscles.
    """

    def __init__(self, suit, side, angle: BiomechanicalAngle):
        """
        Parameters
        ----------
        suit : Teslasuit
            A suit object to send haptic output to
        side : str
            Controlled muscle location side
        angle: BiomechanicalAngle
            Controlled BiomechanicalAngle obj.
        """

        self.agonist_controller = PIDMuscleControler(
            suit, side, angle.agonist_muscle)
        self.antagonist_controller = PIDMuscleControler(
            suit, side, angle.antagonist_muscle)
        self.biomechanical_angle = angle

    def update_haptic_output(self):
        angular_error = self.biomechanical_angle.desired_angle - \
            self.biomechanical_angle.angle
        self.agonist_controller.update_haptic_output(-angular_error)
        self.antagonist_controller.update_haptic_output(angular_error)

    def send_haptic_output(self):
        self.agonist_controller.send_haptic_output()
        self.antagonist_controller.send_haptic_output()


class PIDJointControler():
    """
        A class that creates Joint controller based on Plane controllers.

        Attributes
        ----------
        saggital_plane_controller : PIDPlaneControler
            saggital plane controller obj.
        frontal_plane_controller : PIDPlaneControler
            frontal plane conroller obl.
        transverse_plane_controller: PIDPlaneControler
            transverse plane controller obj.

        Methods
        -------
        update_haptic_output (angular_error):
            Calculates current angular error and updates stimulation output
            for all planes controllers.
        send_haptic_output():
            Sending and playing stimulation with current output values
            on the corresponding to the controlled planes muscles.
    """

    def __init__(self, suit, joint_angle: Joint):
        """
        Parameters
        ----------
        suit : Teslasuit
            A suit object to send haptic output to

        joint_angle: Joint
            Controlled Joint obj.
        """
        if joint_angle.saggital_plane is not None:
            self.saggital_plane_controller = PIDPlaneControler(
                suit, joint_angle.side, joint_angle.saggital_plane)
        else:
            self.saggital_plane_controller = None

        if joint_angle.frontal_plane is not None:
            self.frontal_plane_controller = PIDPlaneControler(
                suit, joint_angle.side, joint_angle.frontal_plane)
        else:
            self.frontal_plane_controller = None

        if joint_angle.transverse_plane is not None:
            self.transverse_plane_controller = PIDPlaneControler(
                suit, joint_angle.side, joint_angle.transverse_plane)
        else:
            self.transverse_plane_controller = None

    def update_haptic_output(self):
        for controller in [self.__dict__[key] for key in iter(self.__dict__)]:
            if controller is not None:
                controller.update_haptic_output()

    def send_haptic_output(self):
        for controller in [self.__dict__[key] for key in iter(self.__dict__)]:
            if controller is not None:
                controller.send_haptic_output()


class PIDSkeletalModelController():
    """
        A class that creates Joint controller based on Plane controllers.

        Attributes
        ----------
        suit : Teslasuit
            A suit object to send stimulation output to

        model: SkeletalModel
            Controlled skeletal model obj.

        Methods
        -------
        update_haptic_output (angular_error):
            Calculates current angular error and updates stimulation output
            for all joint controllers.
        send_haptic_output():
            Sending and playing stimulation with current output values
            on the corresponding to the controlled joint muscles.
    """

    def __init__(self, suit, current_model: SkeletalModel):
        """
        Parameters
        ----------
        suit : Teslasuit
            A suit object to send stimulation output to

        model: SkeletalModel
            Skeletal model obj. to control
        """
        self.suit = suit
        self.model = current_model
        for joint in [self.model.__dict__[key]
                      for key in iter(self.model.__dict__)]:
            joint.controller = PIDJointControler(suit, joint)

    def update_haptic_output(self):
        for joint in [self.model.__dict__[key]
                      for key in iter(self.model.__dict__)]:
            joint.controller.update_haptic_output()

    def send_haptic_output(self):
        for joint in [self.model.__dict__[key]
                      for key in iter(self.model.__dict__)]:
            joint.controller.send_haptic_output()
