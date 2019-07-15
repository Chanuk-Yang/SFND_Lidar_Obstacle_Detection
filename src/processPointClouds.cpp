// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

  	typename pcl::PointCloud<PointT>::Ptr filtered_cloud (new pcl::PointCloud<PointT>);
	typename pcl::VoxelGrid<PointT> sor;
  	sor.setInputCloud (cloud);
  	sor.setLeafSize (filterRes, filterRes, filterRes);
  	sor.filter (*filtered_cloud);

    typename pcl::PointCloud<PointT>::Ptr region_cloud (new pcl::PointCloud<PointT>);
    typename pcl::CropBox<PointT> region (true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filtered_cloud);
    region.filter(*region_cloud);

    std::vector<int> indices;
    typename pcl::CropBox<PointT> roof (true);
    roof.setMin(Eigen::Vector4f (-2.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.7, 1.7, 1, 1));
    roof.setInputCloud(region_cloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (int point : indices)
        inliers->indices.push_back(point);

    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(region_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*region_cloud);



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return region_cloud;

}

template<typename PointT>
std::vector<int> ProcessPointClouds<PointT>::Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::vector<int> inliers_result;

    auto point_bundle = cloud->points;
    
    srand(time(NULL));
    std::cout<<"point number is "<< point_bundle.size()<<std::endl;
    while(maxIterations--){
        std::vector<int> inliers;
        float x_1, y_1, z_1, x_2, y_2, z_2, x_3, y_3, z_3, a, b, c, d;
        while (inliers.size()<3)
            inliers.push_back(rand()%(point_bundle.size()));

        x_1 = point_bundle[inliers[0]].x;
        y_1 = point_bundle[inliers[0]].y;
        z_1 = point_bundle[inliers[0]].z;

        x_2 = point_bundle[inliers[1]].x;
        y_2 = point_bundle[inliers[1]].y;
        z_2 = point_bundle[inliers[1]].z;

        x_3 = point_bundle[inliers[2]].x;
        y_3 = point_bundle[inliers[2]].y;
        z_3 = point_bundle[inliers[2]].z;

        a = (y_2-y_1)*(z_3-z_1) - (z_2-z_1)*(y_3-y_1);
        b = (z_2-z_1)*(x_2-x_1) - (x_2-x_1)*(z_3-z_1);
        c = (x_2-x_1)*(y_3-y_1) - (y_2-y_1)*(x_3-x_1);
        d = -(a*x_1 + b*y_1 + c*z_1);

        for(int i = 0; i<point_bundle.size(); i++){
            PointT point = point_bundle[i];
            float x = point.x;
            float y = point.y;
            float z = point.z;

            float dist = fabs(a*x + b*y + c*z + d)/sqrt(a*a + b*b + c*c);
            if (dist<= distanceTol)
                inliers.push_back(i);
        }
        if (inliers.size()>inliers_result.size())
            inliers_result = inliers;
    }

    return inliers_result;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obst_cloud (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr plane_cloud (new pcl::PointCloud<PointT>);

    for (int indice : inliers->indices)
        plane_cloud->points.push_back(cloud->points[indice]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obst_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obst_cloud, plane_cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // TODO:: Fill in this function to find inliers for the cloud.
    std::vector<int> indices = Ransac3D(cloud, maxIterations, distanceThreshold);
    std::cout<<"size is "<< indices.size()<<std::endl;
    for (int point : indices)
        inliers->indices.push_back(point);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Proximity (int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<bool>& processed, KdTree<PointT>* tree, pcl::PointIndices& cluster, float distanceTol)
{
	
	processed[indice] = true;
	
	cluster.indices.push_back(indice);

	std::vector<int> nearby_points_indices = tree->search(cloud->points[indice], distanceTol);
	for (int single_indice : nearby_points_indices){
		if(!processed[single_indice]){
			Proximity(single_indice, cloud, processed, tree, cluster, distanceTol);
		}
		
	}
}

template<typename PointT>
std::vector<pcl::PointIndices> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT>* tree, float distanceTol, int minSize, int maxSize)
{
	
	std::vector<pcl::PointIndices> clusters;
	std::vector<bool> processed(cloud->points.size(), false);
	
	for (int indice = 0; indice < cloud->points.size() ; ++indice){
		if (!processed[indice]){
			pcl::PointIndices cluster;
			Proximity(indice, cloud, processed, tree, cluster, distanceTol);
            if (cluster.indices.size() > minSize && cluster.indices.size() < maxSize)
			    clusters.push_back(cluster);
		}	
	}
	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // (1) make KDTree

    

    KdTree<PointT>* tree = new KdTree<PointT>();
    for (int i = 0; i < cloud->points.size(); i++){
    	tree->insert(cloud->points[i], i);
    }


    // (2) make Euclidian clustering
    // need to find matching cluster_indices
    

    std::vector<pcl::PointIndices> cluster_indices = euclideanCluster(cloud, tree, clusterTolerance, minSize, maxSize);


    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin() ; it != cluster_indices.end(); it++){
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // project to XY plane
    typename pcl::PointCloud<PointT>::Ptr cluster_2d (new pcl::PointCloud<PointT>);
    cluster_2d->points.resize(cluster->size());
    for (size_t i = 0; i < cluster->points.size(); i++) {
        cluster_2d->points[i].x = cluster->points[i].x;
        cluster_2d->points[i].y = cluster->points[i].y;
        cluster_2d->points[i].z = 0;
    }
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    compute3DCentroid(*cluster_2d, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster_2d, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                    ///    the signs are different and the box doesn't get correctly oriented in some cases.

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    
    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Final transform
    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();


    // Find bounding box for one of the clusters
    BoxQ box;
    box.bboxTransform = bboxTransform;
    box.bboxQuaternion = bboxQuaternion;
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}