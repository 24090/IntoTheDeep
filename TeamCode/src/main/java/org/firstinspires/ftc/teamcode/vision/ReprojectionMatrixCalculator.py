import numpy as np
import scipy

res_scale = 1

focal_length       = (952.90362321, 960.00009409)
principal_point    = (337.90515333, 165.08917237)
rotation_xyz_euler = [50, 0, 0]
degrees            = True
position           = [0, 3.65 , 16.013 - 1.5]
distortion         = [ 8.86386337e-04, 1.89532355e+00, -2.82422652e-02, -7.42910733e-03, -1.11640100e+01]

# find intrinsics matrix
intrinsics_matrix = np.asarray([
						[focal_length[0], 0,               principal_point[0]],
						[0, 	          focal_length[1], principal_point[1]],
						[0,               0,               1/res_scale       ]
])*res_scale

# find extrinsics matrix
rotation_matrix = scipy.spatial.transform.Rotation.from_euler("XYZ", rotation_xyz_euler, degrees).as_matrix()
translation_vector = np.asarray(position).reshape((3,1))
homogenous_transformation_matrix = np.linalg.inv(
	np.vstack((
		np.hstack((rotation_matrix, translation_vector)),
		          [0,     0,     0,          1        ]
	))
)
extrinsics_matrix = homogenous_transformation_matrix[:-1]

# find reprojection matrix

projection_matrix = (intrinsics_matrix @ extrinsics_matrix)
reprojection_matrix = np.linalg.inv(projection_matrix[:, [0,1,3]])
# generate code to make it
code = "// generated with ReprojectionMatrixCalculator.py\n"
for row_num in range(3):
	code += f"\t\trm.put({row_num}, 0, {reprojection_matrix[row_num][0]: e}); rm.put({row_num}, 1, {reprojection_matrix[row_num][1]: e}); rm.put({row_num}, 2, {reprojection_matrix[row_num][2]: e});\n"
code += "\n"
for row_num in range(3):
	code += f"\t\tcm.put({row_num}, 0, {intrinsics_matrix[row_num][0]: e}); cm.put({row_num}, 1, {intrinsics_matrix[row_num][1]: e}); cm.put({row_num}, 2, {intrinsics_matrix[row_num][2]: e});\n"
code += "\n"
for col_num, value in enumerate(distortion):
	code += f"\t\tdc.put(0, {col_num}, {value: e});\n"
code += "\t\t// end"
print(code)

# test functions
def project(x, y, z):
	v = projection_matrix @ [x, y, z, 1]
	v = v/v[2]
	return v

def reproject(u, v):
	v = reprojection_matrix @ [u, v, 1]
	v = v/v[2]
	return v

text = open("Camera.java", "r").read()
text = text.replace("// generated with ReprojectionMatrixCalculator.py", "RMC_SPLIT")
text = text.replace("// end", "RMC_SPLIT")
text = text.split("RMC_SPLIT")
text[1] = code
print(code)
if (input("autoreplace? ") == "y"):
	open("Camera.java", "w").write("".join(text))
