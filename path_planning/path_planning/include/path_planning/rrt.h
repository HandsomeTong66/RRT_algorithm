#ifndef rrt_h
#define rrt_h

#include <vector>
using namespace std;
namespace rrt {
	class RRT{

        public:

            RRT();
            RRT(double input_PosX, double input_PosY, double input_PosZ);

            struct rrtNode{
                int nodeID;
                double posX;
                double posY;

                //二维改三维
                double posZ;
                int parentID;
                //vector<int> children;//没用到
            };

            vector<rrtNode> getTree();
            void setTree(vector<rrtNode> input_rrtTree);
            int getTreeSize();

            void addNewNode(rrtNode node);
            rrtNode removeNode(int nodeID);
            rrtNode getNode(int nodeID);

            double getPosX(int nodeID);
            double getPosY(int nodeID);
            //二维改三维
            double getPosZ(int nodeID);
            void setPosX(int nodeID, double input_PosX);
            void setPosY(int nodeID, double input_PosY);

            rrtNode getParent(int nodeID);
            void setParentID(int nodeID, int parentID);

            //void addChildID(int nodeID, int childID);
            //vector<int> getChildren(int nodeID);
            //int getChildrenSize(int nodeID);
            
            //三维
            int getNearestNodeID(double X, double Y, double Z);
            vector<int> getRootToEndPath(int endNodeID);

        private:
            vector<rrtNode> rrtTree;
            //三维
            double getEuclideanDistance(double sourceX, double sourceY, double sourceZ, double destinationX, double destinationY, double destinationZ);
	};
};

#endif
