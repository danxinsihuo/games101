#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
	SplitMethod splitMethod)
	: maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
	primitives(std::move(p))
{
	time_t start, stop;
	time(&start);
	if (primitives.empty())
		return;

	root = recursiveBuild(primitives);

	time(&stop);
	double diff = difftime(stop, start);
	int hrs = (int)diff / 3600;
	int mins = ((int)diff / 60) - (hrs * 60);
	int secs = (int)diff - (hrs * 3600) - (mins * 60);

	printf(
		"\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
		hrs, mins, secs);
}

// 递归的建立BVH二叉树
// 1.先将场景中所有基元合并成根节点的包围盒 -> 这个并没有用上
// 2.如果物体数量为1，则创建一个叶子节点，并将该物体作为该节点的对象。
// 3.如果物体数量为2，则递归地创建左右子节点(这两个都是叶子节点)，并将子节点的包围盒合并成该节点的包围盒
// 4.如果物体数量大于2，则计算所有基元的中心点，创建一个中心点的包围盒，判断这个包围盒上哪个轴包围的区域最大，
// 并沿着那个维度进行对物体排序。然后将物体分成左右两个部分，分别递归左、右边物体集合
BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
	BVHBuildNode* node = new BVHBuildNode();

	// Compute bounds of all primitives in BVH node
	Bounds3 bounds;
	for (int i = 0; i < objects.size(); ++i)
		bounds = Union(bounds, objects[i]->getBounds());
	if (objects.size() == 1) {
		// Create leaf _BVHBuildNode_
		node->bounds = objects[0]->getBounds();
		node->object = objects[0];
		node->left = nullptr;
		node->right = nullptr;
		return node;
	}
	else if (objects.size() == 2) {
		node->left = recursiveBuild(std::vector{ objects[0] });
		node->right = recursiveBuild(std::vector{ objects[1] });

		node->bounds = Union(node->left->bounds, node->right->bounds);
		return node;
	}
	else {
		Bounds3 centroidBounds;
		for (int i = 0; i < objects.size(); ++i)
			centroidBounds =
			Union(centroidBounds, objects[i]->getBounds().Centroid());
		int dim = centroidBounds.maxExtent();
		switch (dim) {
			case 0:
				std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
					return f1->getBounds().Centroid().x <
						f2->getBounds().Centroid().x;
					});
				break;
			case 1:
				std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
					return f1->getBounds().Centroid().y <
						f2->getBounds().Centroid().y;
					});
				break;
			case 2:
				std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
					return f1->getBounds().Centroid().z <
						f2->getBounds().Centroid().z;
					});
				break;
		}

		auto beginning = objects.begin();
		auto middling = objects.begin() + (objects.size() / 2);
		auto ending = objects.end();

		auto leftshapes = std::vector<Object*>(beginning, middling);
		auto rightshapes = std::vector<Object*>(middling, ending);

		assert(objects.size() == (leftshapes.size() + rightshapes.size()));

		node->left = recursiveBuild(leftshapes);
		node->right = recursiveBuild(rightshapes);

		node->bounds = Union(node->left->bounds, node->right->bounds);
	}

	return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
	Intersection isect;
	if (!root)
		return isect;
	isect = BVHAccel::getIntersection(root, ray);
	return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
	// TODO Traverse the BVH to find intersection
	std::array<int, 3> dirIsNeg;
	dirIsNeg[0] = ray.direction[0] > 0;
	dirIsNeg[1] = ray.direction.y > 0;
	dirIsNeg[2] = ray.direction.z > 0;
	// {ray.direction[0] > 0, ray.direction[1] > 0, ray.direction[2] > 0};

	// 空对象，供不相交时返回
	Intersection interNull;
	// 递归的判断在光线是否与一个BVH节点和它的子节点（包围盒）相交，如果与父节点相交，
	// 子节点都有可能相交,直到叶子节点，此时应该判断是否和物体相交

	// 先判断相交,再判断是哪种节点
	if (node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
		// 叶子节点
		if (node->left == nullptr && node->right == nullptr)
			return node->object->getIntersection(ray);
	// 父节点
		else
		{
			Intersection l = getIntersection(node->left, ray);
			Intersection r = getIntersection(node->right, ray);

			// 返回深度浅的物体的相交信息
			return l.distance < r.distance ? l : r;
		}
	else
		return interNull;
}