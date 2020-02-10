import bpy, bmesh, math

initial_pos = [0,0,0]



bone_index = { "SpineBase" : 0,
    "SpineMid" : 1,
    "Neck" : 2,
    "Head" : 3,
    "ShoulderLeft" : 4,
    "ElbowLeft" : 5,
    "WristLeft" : 6,
    "HandLeft" : 7,
    "ShoulderRight" : 8,
    "ElbowRight" : 9,
    "WristRight" : 10,
    "HandRight" : 11,
    "HipLeft" : 12,
    "KneeLeft" : 13,
    "AnkleLeft" : 14,
    "FootLeft" : 15,
    "HipRight" : 16,
    "KneeRight" : 17,
    "AnkleRight" : 18,
    "FootRight" : 19 }

# Initialize now and set later
bones = { "SpineBase" : 0,
    "SpineMid" : 1,
    "Neck" : 2,
    "Head" : 3,
    "ShoulderLeft" : 4,
    "ElbowLeft" : 5,
    "WristLeft" : 6,
    "HandLeft" : 7,
    "ShoulderRight" : 8,
    "ElbowRight" : 9,
    "WristRight" : 10,
    "HandRight" : 11,
    "HipLeft" : 12,
    "KneeLeft" : 13,
    "AnkleLeft" : 14,
    "FootLeft" : 15,
    "HipRight" : 16,
    "KneeRight" : 17,
    "AnkleRight" : 18,
    "FootRight" : 19 }

bone_length = { "SpineBase" : 0,
    "SpineMid" : 1,
    "Neck" : 2,
    "Head" : 3,
    "ShoulderLeft" : 4,
    "ElbowLeft" : 5,
    "WristLeft" : 6,
    "HandLeft" : 7,
    "ShoulderRight" : 8,
    "ElbowRight" : 9,
    "WristRight" : 10,
    "HandRight" : 11,
    "HipLeft" : 12,
    "KneeLeft" : 13,
    "AnkleLeft" : 14,
    "FootLeft" : 15,
    "HipRight" : 16,
    "KneeRight" : 17,
    "AnkleRight" : 18,
    "FootRight" : 19 }

bone_quat = { "SpineBase" : [0,0,0,0],
    "SpineMid" : [0,0,0,0],
    "Neck" : [0,0,0,0],
    "Head" : [0,0,0,0],
    "ShoulderLeft" : [0,0,0,0],
    "ElbowLeft" : [0,0,0,0],
    "WristLeft" : [0,0,0,0],
    "HandLeft" : [0,0,0,0],
    "ShoulderRight" : [0,0,0,0],
    "ElbowRight" : [0,0,0,0],
    "WristRight" : [0,0,0,0],
    "HandRight" : [0,0,0,0],
    "HipLeft" : [0,0,0,0],
    "KneeLeft" : [0,0,0,0],
    "AnkleLeft" : [0,0,0,0],
    "FootLeft" : [0,0,0,0],
    "HipRight" : [0,0,0,0],
    "KneeRight" : [0,0,0,0],
    "AnkleRight" : [0,0,0,0],
    "FootRight" : [0,0,0,0] }

bone_pos = { "SpineBase" : [0,0,0],
    "SpineMid" : [0,0,0],
    "Neck" : [0,0,0],
    "Head" : [0,0,0],
    "ShoulderLeft" : [0,0,0],
    "ElbowLeft" : [0,0,0],
    "WristLeft" : [0,0,0],
    "HandLeft" : [0,0,0],
    "ShoulderRight" : [0,0,0],
    "ElbowRight" : [0,0,0],
    "WristRight" : [0,0,0],
    "HandRight" : [0,0,0],
    "HipLeft" : [0,0,0],
    "KneeLeft" : [0,0,0],
    "AnkleLeft" : [0,0,0],
    "FootLeft" : [0,0,0],
    "HipRight" : [0,0,0],
    "KneeRight" : [0,0,0],
    "AnkleRight" : [0,0,0],
    "FootRight" : [0,0,0] }

bone_parent = { "SpineBase" : "origin",
    "SpineMid" : "SpineBase",
    "Neck" : "SpineMid",
    "Head" : "Neck",
    "ShoulderLeft" : "SpineMid",
    "ElbowLeft" : "ShoulderLeft",
    "WristLeft" : "ElbowLeft",
    "HandLeft" : "WristLeft",
    "ShoulderRight" : "SpineMid",
    "ElbowRight" : "ShoulderRight",
    "WristRight" : "ElbowRight",
    "HandRight" : "WristRight",
    "HipLeft" : "SpineBase",
    "KneeLeft" : "HipLeft",
    "AnkleLeft" : "KneeLeft",
    "FootLeft" : "AnkleLeft",
    "HipRight" : "SpineBase",
    "KneeRight" : "HipRight",
    "AnkleRight" : "KneeRight",
    "FootRight" : "AnkleRight" }

def read_some_data(context, filepath, use_some_setting):
    print("running read_some_data...")
    f = open(filepath, 'r', encoding='utf-8')
    data = f.read()
    f.close()

    # would normally load the data here
    #print(data)
    
    # Add the new armature,
    scene = bpy.context.scene
    for obj in scene.objects:
        obj.select_set(False)

    arm_obj = bpy.data.armatures.new("Jimbob")
    jimbo = bpy.data.objects.new("Jimbob", arm_obj);
    
    bpy.context.collection.objects.link(jimbo)
    
    jimbo.select_set(True)
    
    # must be in edit mode to add bones
    bpy.ops.object.mode_set(mode='OBJECT', toggle=False)
    bpy.ops.object.mode_set(mode='EDIT', toggle=False)
    
    edit_bones = arm_obj.edit_bones


    frame_id = -1 #magic number
    for data_line in data.splitlines():
        pose_values = data_line.split(',')
    
        #print(pose_values)


        # Create Armature Only For First Frame
        if (frame_id == -1):
            frame_id = 0;

            for bone_name in bone_index:
                idx = bone_index[bone_name]*3

                bone_pos[bone_name] = [float(pose_values[idx]),float(pose_values[idx+1]),float(pose_values[idx+2])]

                bones[bone_name] = edit_bones.new(bone_name)
                
                print(bone_pos[bone_name])

                bone_length[bone_name] = calc_length(bone_pos[bone_name])

                if bone_name is "SpineBase":
                    bones[bone_name].head = [initial_pos[0], initial_pos[1], initial_pos[2]]

                else:

                    # all bone positions are relative to parent (for the first frame only)
                    if (bone_name is not "HipRight" and bone_name is not "HipLeft"):
                        for i in range(0,len(bone_pos[bone_name])):
                            bone_pos[bone_name][i] = bone_pos[bone_name][i] + bone_pos[bone_parent[bone_name]][i]

                    bones[bone_name].parent = bones[bone_parent[bone_name]]

                    if (bone_name is "HipRight" or bone_name is "HipLeft"):
                        print(bone_name)
                        bones[bone_name].head = (initial_pos[0], initial_pos[1], initial_pos[2])
                    else:
                        bones[bone_name].head = (bone_pos[bone_parent[bone_name]][0], bone_pos[bone_parent[bone_name]][1], bone_pos[bone_parent[bone_name]][2])
                
                bones[bone_name].tail = (bone_pos[bone_name][0],bone_pos[bone_name][1],bone_pos[bone_name][2])


            
        else:
            bpy.ops.object.mode_set(mode='POSE', toggle=False)

            for c_bone in jimbo.pose.bones:
                bone_name = c_bone.name

                if (frame_id == 0):
                    print(bone_name)
                    print(c_bone.rotation_quaternion)

                idx = bone_index[bone_name]*4

                # parse pose values (quats, 0 index is w)
                bone_quat[bone_name] = [float(pose_values[idx]),float(pose_values[idx+1]),float(pose_values[idx+2]),float(pose_values[idx+3])]

                # for each bone, rotate the vertical pose vector (0,1,0)*bone_length by the quat, gives us new pose
                original_pose = [0, bone_length[bone_name], 0]
                new_pose = quat_rotate(original_pose, bone_quat[bone_name])


#                

                c_bone.rotation_quaternion = bone_quat[bone_name]




                # if "SpineBase" in bone_name:
                #     c_bone.tail = (new_pose[0],new_pose[1],new_pose[2])

                # else:
                #     c_bone.tail = (bone_pos[bone_parent[bone_name]][0] + new_pose[0],
                #                                 bone_pos[bone_parent[bone_name]][1] + new_pose[1],
                #                                 bone_pos[bone_parent[bone_name]][2] + new_pose[2])

                # # base is tail of parent (except for legs), tail is base + new vector?
                # if bone_name is "SpineBase":
                #     bones[bone_name].head = [initial_pos[0], initial_pos[1], initial_pos[2]]

                #     bones[bone_name].tail = (new_pose[0],new_pose[1],new_pose[2])
                # else:
                #     if (bone_name is "HipRight" or bone_name is "HipLeft"):
                #         bones[bone_name].head = (initial_pos[0], initial_pos[1], initial_pos[2])
                #     else:
                #         bones[bone_name].head = (bone_pos[bone_parent[bone_name]][0], bone_pos[bone_parent[bone_name]][1], bone_pos[bone_parent[bone_name]][2])
                
                #     bones[bone_name].tail = (bone_pos[bone_parent[bone_name]][0] + new_pose[0],
                #                             bone_pos[bone_parent[bone_name]][1] + new_pose[1],
                #                             bone_pos[bone_parent[bone_name]][2] + new_pose[2])


                #add keyframe
                #bones[bone_name].keyframe_insert(data_path="head", frame=frame_id)
                c_bone.keyframe_insert(data_path="rotation_quaternion", frame=frame_id)

            frame_id = frame_id+1


    return {'FINISHED'}


# ImportHelper is a helper class, defines filename and
# invoke() function which calls the file selector.
from bpy_extras.io_utils import ImportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy.types import Operator


class ImportSomeData(Operator, ImportHelper):
    """This appears in the tooltip of the operator and in the generated docs"""
    bl_idname = "import_test.some_data"  # important since its how bpy.ops.import_test.some_data is constructed
    bl_label = "Import Some Data"

    # ImportHelper mixin class uses this
    filename_ext = ".csv"

    filter_glob: StringProperty(
        default="*.csv",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )

    # List of operator properties, the attributes will be assigned
    # to the class instance from the operator settings before calling.
    use_setting: BoolProperty(
        name="Example Boolean",
        description="Example Tooltip",
        default=True,
    )

    type: EnumProperty(
        name="Example Enum",
        description="Choose between two items",
        items=(
            ('OPT_A', "First Option", "Description one"),
            ('OPT_B', "Second Option", "Description two"),
        ),
        default='OPT_A',
    )

    def execute(self, context):
        return read_some_data(context, self.filepath, self.use_setting)


# Only needed if you want to add into a dynamic menu
def menu_func_import(self, context):
    self.layout.operator(ImportSomeData.bl_idname, text="Text Import Operator")


def register():
    bpy.utils.register_class(ImportSomeData)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)


def unregister():
    bpy.utils.unregister_class(ImportSomeData)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)



def calc_length(vector):
    return math.sqrt( vector[0]**2 + vector[1]**2 + vector[2]**2)


# quat[1:3] - vector, quat[0] - w
def quat_rotate(vector, quat):
    new_v = [0,0,0]

    new_v[0] =     (1 - 2*quat[2]**2 - 2*quat[3]**2) * vector[0] + 2*(quat[1]*quat[2] + quat[0]*quat[3]) * vector[1] + 2*(quat[1]*quat[3] - quat[0]*quat[2]) * vector[2]
    new_v[1] = 2*(quat[1]*quat[2] - quat[0]*quat[3]) * vector[0] +     (1 - 2*quat[1]**2 - 2*quat[3]**2) * vector[1] + 2*(quat[2]*quat[3] + quat[0]*quat[1]) * vector[2]
    new_v[2] = 2*(quat[1]*quat[3] + quat[0]*quat[2]) * vector[0] + 2*(quat[2]*quat[3] - quat[0]*quat[1]) * vector[1] +     (1 - 2*quat[1]**2 - 2*quat[2]**2) * vector[2]

    return new_v

def vec_normalize (vec):

    mag = calc_length(vec)

    vec[0] = vec[0]/mag;
    vec[1] = vec[1]/mag;
    vec[2] = vec[2]/mag;

    return vec


def quat_from_axis_angle( axis, rad):
    q = [0,0,0,0]

    norm = vec_normalize(axis);
    a = rad * 0.5
    s = math.sin(a)

    q[0] = math.cos(a)
    q[1] = norm[0] * s;
    q[2] = norm[1] * s;
    q[3] = norm[2] * s;
        
    return q


if __name__ == "__main__":
    register()

    # test call
    bpy.ops.import_test.some_data('INVOKE_DEFAULT')
