#ifndef GATLAS_HPP
#define GATLAS_HPP

#include <unordered_map>
#include <set>
#include <vector>
#include <mutex>
#include <Eigen/Dense>


/**
 * This class manages the map of transformation 
 * bewteen fiducial markers.
 */


class GAtlas {
    public: 
        GAtlas();
        ~GAtlas();

		/**
		 * Insert a transformation between two nodes in the Atlas
		 * a -- TR --> b
		 */
        void insert(int a, int b, const TransformData& TR);

		/**
		 * Retrieve the transformation between two nodes in the Atlas
		 */
        bool getTransform(int id_origin, int id_dest, TransformData& tr);

		/**
		 * Get the vector of fiducial markers Ids in the Atlas
		 */
        std::vector<int> get_fiducialsIds();

    private:
		/**
		 * Mutex
		 */
		std::mutex data_mx;

		/**
		 * Data structure representing the Atlas
		 */
        std::unordered_map<int, AtlasElement> _gatlas;

		/**
		 * Variable used during the searches in the Atlas
		 */
        std::set<int> _visited;

		/**
		 * Find a path between two nodes in the Atlas
		 */
        bool find_path(int s, int e, TransformData& TR);
};


#endif
