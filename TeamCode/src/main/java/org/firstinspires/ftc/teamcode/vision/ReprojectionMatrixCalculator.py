import numpy as np
import scipy

focal_length       = (960.04759489, 955.66073989)
principal_point    = (285.88567653, 259.32328781)
rotation_xyz_euler = [50, 0, 0]
degrees            = True
position           = [-2.65, 3.31 + 1.34, 15.55 - 1.5]
distortion         = [ 4.84124426e-02,  1.12119387e+00,  7.82766988e-03, -4.26765995e-04, -8.51215210e+00]
res_scale = 1

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

projection_matrix = (intrinsics_matrix @ extrinsics_matrix)[:, [0,1,3]]
reprojection_matrix = np.linalg.inv(projection_matrix)
print(projection_matrix)
# generate code to make it
code = "// generated with ReprojectionMatrixCalculator.py\n"
for row_num in range(3):
	code += f"\t\tpm.put({row_num}, 0, {projection_matrix[row_num][0]: e}); pm.put({row_num}, 1, {projection_matrix[row_num][1]: e}); pm.put({row_num}, 2, {projection_matrix[row_num][2]: e});\n"
code += "\n"
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
def project(x, y):
	v = projection_matrix @ [x, y, 1]
	v = v/v[2]
	return v

def reproject(u, v):
	v = reprojection_matrix @ [u, v, 1]
	v = v/v[2]
	return v

print(project(1, 20))

text = open("Camera.java", "r").read()
text = text.replace("// generated with ReprojectionMatrixCalculator.py", "RMC_SPLIT")
text = text.replace("// end", "RMC_SPLIT")
text = text.split("RMC_SPLIT")
text[1] = code
print(code)
if (input("autoreplace? ") == "y"):
	open("Camera.java", "w").write("".join(text))
