from typing import Tuple
import numpy as np
from scipy.spatial import KDTree
from scipy.spatial.distance import cdist
from sklearn.neighbors import NearestNeighbors

import utils


def q1_a(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Fit a least squares plane by taking the Eigen values and vectors
    of the sample covariance matrix

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space

    Returns
    -------
    normal : np.ndarray
        array of shape (3,) denoting surface normal of the fitting plane
    center : np.ndarray
        array of shape (3,) denoting center of the points
    '''

    # Calculate the sample mean of the points
    center = np.mean(P, axis=0)

    # Calculate the sample covariance matrix
    covariance_mat = np.cov(P.T)

    # Obtain the eigenvalues and eigenvectors of the covariance matrix
    eigenvalues, eigenvectors = np.linalg.eig(covariance_mat)

    # The eigenvector with the smallest eigenvalue corresponds to the surface normal of the plane
    normal = eigenvectors[:, np.argmin(eigenvalues)]

    print("normal = ", normal)

    return normal, center
    pass


def q1_c(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Fit a plane using RANSAC

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space

    Returns
    -------
    normal : np.ndarray
        array of shape (3,) denoting surface normal of the fitting plane
    center : np.ndarray
        array of shape (3,) denoting center of the points
    '''

    # Define hyperparameters
    MAX_ITER = 1000
    EPSILON = 0.05

    # Initialize best fitting plane
    best_score = 0
    best_normal = [0, 0, 0]
    best_center = [0, 0, 0]

    # RANSAC loop
    for i in range(MAX_ITER):
        # Randomly select n points
        sample = np.random.choice(P.shape[0], 3, replace=False)
        sample_points = P[sample, :]
        point_A = sample_points[0]
        point_B = sample_points[1]
        point_C = sample_points[2]

        # Compute plane normal and center
        normal = np.cross(point_B - point_A, point_C - point_A)
        unit_normal_vector = normal / np.linalg.norm(normal)
        center = np.mean(sample_points, axis=0)

        # Compute projected_vector and euclidean distance to plane for each point
        projected_vector = np.dot(P - center, unit_normal_vector)
        euclidean_dists = np.abs(projected_vector)

        # Count inliers
        inliers = np.count_nonzero(euclidean_dists < EPSILON)

        # Update best score and fitting plane
        if inliers > best_score:
            print("inliers = ", inliers)
            best_normal = normal
            best_center = center
            best_score = inliers

    return best_normal, best_center

    pass


def q2(P: np.ndarray, N: np.ndarray) -> Tuple[np.ndarray, float]:
    '''
    Localize a sphere in the point cloud. Given a point cloud as
    input, this function should locate the position and radius
    of a sphere

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space
    N : np.ndarray
        Nx3 matrix denoting normals of pointcloud

    Returns
    -------
    center : np.ndarray
        array of shape (3,) denoting sphere center
    radius : float
        scalar radius of sphere
    '''

    MAX_ITER = 10000
    EPSILON = 0.001  #Error tolerance for fitness calculation
    max_inliers = 0  #will update this as we get more and more inliers within the sphere.
    final_center = [0, 0, 0]
    final_radius = float(0)

    for i in range(MAX_ITER):

        # sample a point from point cloud P and
        sample = np.random.randint(0, len(P))
        sample_pi = P[sample]
        sample_normal = N[sample]
        direction = sample_normal - sample_pi

        #take unit vector of normal that is n hat
        unit_vector = direction / np.linalg.norm(direction)

        random_radius = np.random.uniform(0.05, 0.11)
        #print("rad = ", random_radius)

        #candidate center will be in the direction of the associated surface normal
        candidate_center = sample_pi + random_radius * unit_vector

        #euclidean_distance = np.sqrt(sample_pi[0] * candidate_center[0] + sample_pi[1] * candidate_center[1] + sample_pi[2] * candidate_center[2])

        euclidean_distance = np.linalg.norm(P - candidate_center, axis=1)
        #print("euclidean_distance = ", euclidean_distance)
        #printed above statement to chcek all the euclidean distances. so that I can set the value of epsilon

        corresponding_inliers = ((random_radius - EPSILON < euclidean_distance) & (euclidean_distance < EPSILON + random_radius))
        candidate_inliers = np.count_nonzero(corresponding_inliers)
        #candidate_inliers = np.sum(corresponding_inliers)

        # Update max inliers per iteration and it's corresponding center and radius
        if candidate_inliers > max_inliers:
            print("inside")
            max_inliers = candidate_inliers
            final_center = candidate_center
            final_radius = random_radius

    print("final_center = ", final_center)
    print("final_radius = ", final_radius)
    print("max_inliers = ", max_inliers)
    #finally return the center and radius
    return final_center, final_radius

    pass


def q3(P: np.ndarray, N: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    '''
    Localize a cylinder in the point cloud. Given a point cloud as
    input, this function should locate the position, orientation,
    and radius of the cylinder

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting 100 points in 3D space
    N : np.ndarray
        Nx3 matrix denoting normals of pointcloud

    Returns
    -------
    center : np.ndarray
        array of shape (3,) denoting cylinder center
    axis : np.ndarray
        array of shape (3,) pointing along cylinder axis
    radius : float
        scalar radius of cylinder
    '''
    MAX_ITER = 10000
    EPSILON = 0.008
    max_inliers = 0
    final_center = [0, 0, 0]
    final_axis = [0, 0, 0]
    final_radius = float(0)

    for i in range(MAX_ITER):
        # sample 2 differenr random points and it's respective normal unit vector(n hat)
        sample = np.random.choice(P.shape[0], 2, replace=False)
        sample_p1, sample_p2 = P[sample]

        #sample random radius
        random_radius = np.random.uniform(0.05, 0.10)

        sample_N1 = N[sample[0]]
        sample_N2 = N[sample[1]]

        #setting the cylinder axis direction
        axis_unit_vector = np.cross(sample_N1, sample_N2)
        axis_unit_vector = axis_unit_vector / np.linalg.norm(axis_unit_vector)

        #N1_unit_vector = sample_N1 / np.linalg.norm(sample_N1)


        #pick candidate center as one of the sample points
        #candidate_center = sample_p1
        candidate_center = sample_p1 + random_radius * sample_N1

        # Step 5: Project the points in the cloud onto the plane orthogonal to the axis of the cylinder
        projection_mat = np.eye(3) - np.outer(axis_unit_vector, axis_unit_vector)

        #projected_center = candidate_center @ projection_mat
        #projected_center = candidate_center.dot(projection_mat)

        #projected_center = np.dot(candidate_center, projection_mat)
        projected_center = np.matmul(projection_mat, candidate_center.T)

        #print("shape C = ", projected_center.shape)
        # projected_point_cloud = np.dot(P, projection_mat)
        #projected_point_cloud = projection_mat @ P.T

        #projected_point_cloud = P.dot(projection_mat)
        projected_point_cloud = np.matmul(projection_mat, P.T)
        #print("shape P = ", projected_point_cloud.shape)

        # Evaluate the number of inliers
        euclidean_distances = np.linalg.norm(projected_point_cloud.T - projected_center, axis=1)
        corresponding_inliers = ((random_radius - EPSILON < euclidean_distances) & (euclidean_distances < EPSILON + random_radius))
        candidate_inliers = np.count_nonzero(corresponding_inliers)

        if candidate_inliers > max_inliers:
            print("inside")
            max_inliers = candidate_inliers
            final_axis = axis_unit_vector
            final_radius = random_radius
            final_center = projected_center

    print("final_center = ", final_center)
    print("final_axis = ", final_axis)
    print("final_radius = ", final_radius)
    print("max_inliers = ", max_inliers)

    return final_center, final_axis, final_radius
    pass


def q4_a(M: np.ndarray, D: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Find transformation T such that D = T @ M. This assumes that M and D are
    corresponding (i.e. M[i] and D[i] correspond to same point)

    Attributes
    ----------
    M : np.ndarray
        Nx3 matrix of points
    D : np.ndarray
        Nx3 matrix of points

    Returns
    -------
    T : np.ndarray
        4x4 homogenous transformation matrix

    Hint
    ----
    use `np.linalg.svd` to perform singular value decomposition
    '''

    #find mean
    mean_M = np.mean(M, axis=0)
    mean_D = np.mean(D, axis=0)

    # Center M and D around their means
    M_center = M - mean_M
    D_center = D - mean_D

    # print(" M_center = ", M_center)
    # print(" D_center = ", D_center)

    # Compute covariance matrix H
    transformation_mat = np.dot(M_center.T, D_center)

    print("transformation_mat = ", transformation_mat)

    # Perform singular value decomposition on Tranformation mat
    U, s, Vt = np.linalg.svd(transformation_mat)

    # Construct rotation matrix R and translation vector t
    Rot_mat = np.dot(Vt.T, U.T)
    t_mat = mean_D.T - np.dot(Rot_mat, mean_M.T)

    print("Rot = ", Rot_mat)
    print("t_mat = ", t_mat)

    # Construct homogeneous transformation matrix T
    homogeneous_mat = np.identity(4)
    homogeneous_mat[:3, :3] = Rot_mat
    homogeneous_mat[:3, 3] = t_mat

    return homogeneous_mat
    pass


def q4_c(M: np.ndarray, D: np.ndarray) -> np.ndarray:
    '''
    Solves iterative closest point (ICP) to generate transformation T to best
    align the points clouds: D = T @ M

    Attributes
    ----------
    M : np.ndarray
        Nx3 matrix of points
    D : np.ndarray
        Nx3 matrix of points

    Returns
    -------
    T : np.ndarray
        4x4 homogenous transformation matrix

    Hint
    ----
    you should make use of the function `q4_a`
    '''
    close_M = np.ones((4, M.shape[0]))
    close_D = np.ones((4, D.shape[0]))
    #Constants
    MAX_ITER = 50 # feel free to change these values to obtain good result
    EPSILON = 0.001 # if you increase the value of epsilon, might not be able to get good results


    #slicing to copy the M and D point clouds
    close_M[:3, :] = np.copy(M.T)
    close_D[:3, :] = np.copy(D.T)

    # error function explained in slide no 38
    minimal_value = 0

    for i in range(MAX_ITER):

        #print("i = ", i)

        closest_sample = NearestNeighbors(n_neighbors=1)
        closest_sample.fit(close_D[:3, :].T)
        euclidean_distances, indices = closest_sample.kneighbors(close_M[:3, :].T, return_distance=True)

        #it returns flatten 1 D array
        indices = indices.ravel()

        # compute the transformation between the current source and nearest destination points
        T = q4_a(close_M[:3, :].T, close_D[:3, indices].T)

        #print("shape of T = ", T.shape)
        # update the current source
        close_M = np.dot(T, close_M)

        #print("close_M = ", close_M)
        #chceking minimial value of error
        mean_error = np.mean(euclidean_distances)
        if np.abs(minimal_value - mean_error) < EPSILON: # error function explained in slide no 38
            break
        minimal_value = mean_error
        #Updating the minimal value in above step

    # calculate final transformation
    homogeneous_mat = q4_a(M, close_M[:3, :].T)

    return homogeneous_mat


    '''
    Another approach that didn't work well
    # Set maximum number of iterations
    MAX_ITER = 20
    EPSILON = 0.001
    # Iterate until convergence or maximum iterations
    for i in range(MAX_ITER):

        # Find closest neighbors of each point in X in P
        distances = np.sqrt(np.sum((M[:, np.newaxis] - D) ** 2, axis=2))
        correspondences = np.argmin(distances, axis=1)

        # Compute transformation that best aligns X to P
        Homogeneous_mat = q4_a(M, D[correspondences])
        R = Homogeneous_mat[:3, :3]
        t = Homogeneous_mat[:3, 3] - np.dot(R, np.mean(M, axis=0))


        # Apply transformation to P
        M = np.dot(R, M.T).T + t.T
        # source = M
        # source = np.matmul(Homogeneous_mat[:3, :3], source.T).T + Homogeneous_mat[:3, 3].T
        # M = source
        # Check for convergence
        if np.linalg.norm(t) < 1e-6 and np.max(np.abs(R - np.eye(M.shape[1]))) < 1e-6:
            print("converged")
            break

        # Check convergence
        # if np.allclose(Homogeneous_mat, np.identity(4)):
        #     print("converged")
        #     print(Homogeneous_mat)
        #     break

    return Homogeneous_mat
    '''
    pass
