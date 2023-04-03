def q1_c(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    MAX_ITER = 1000
    EPSILON = 0.01
    best_normal = [0, 0, 0]
    best_center = [0, 0, 0]
    best_inliers = np.array([])

    for i in range(MAX_ITER):
        # Randomly select three points from the point cloud
        indices = np.random.choice(P.shape[0], size=3, replace=False)
        points = P[indices, :]

        # Compute the normal vector of the plane defined by the three points
        v1 = points[1, :] - points[0, :]
        v2 = points[2, :] - points[0, :]
        normal = np.cross(v1, v2)
        normal /= np.linalg.norm(normal)

        # Compute the center of the points
        center = np.mean(P, axis=0)

        # Compute the distance of each point to the plane
        distances = np.abs(np.dot(P - center, normal))

        # Determine the inliers based on the distance threshold
        inliers = np.where(distances < EPSILON)[0]

        # If the current set of inliers is better than the previous best set,
        # update the best set of inliers and the corresponding plane parameters
        if inliers.shape[0] > best_inliers.shape[0]:
            best_normal = normal
            best_center = center
            best_inliers = inliers

    # Refit the plane using all of the inliers
    inlier_points = P[best_inliers, :]
    best_center = np.mean(inlier_points, axis=0)

    # Calculate the sample covariance matrix of the inliers
    cov_mat = np.cov(inlier_points.T)

    # Obtain the eigenvalues and eigenvectors of the covariance matrix
    eigenvalues, eigenvectors = np.linalg.eig(cov_mat)

    # The eigenvector with the smallest eigenvalue corresponds to the surface normal of the plane
    best_normal = eigenvectors[:, np.argmin(eigenvalues)]

    return best_normal, best_center

    pass