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

// �ݹ�Ľ���BVH������
// 1.�Ƚ����������л�Ԫ�ϲ��ɸ��ڵ�İ�Χ�� -> �����û������
// 2.�����������Ϊ1���򴴽�һ��Ҷ�ӽڵ㣬������������Ϊ�ýڵ�Ķ���
// 3.�����������Ϊ2����ݹ�ش��������ӽڵ�(����������Ҷ�ӽڵ�)�������ӽڵ�İ�Χ�кϲ��ɸýڵ�İ�Χ��
// 4.���������������2����������л�Ԫ�����ĵ㣬����һ�����ĵ�İ�Χ�У��ж������Χ�����ĸ����Χ���������
// �������Ǹ�ά�Ƚ��ж���������Ȼ������ֳ������������֣��ֱ�ݹ����ұ����弯��
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

	// �ն��󣬹����ཻʱ����
	Intersection interNull;
	// �ݹ���ж��ڹ����Ƿ���һ��BVH�ڵ�������ӽڵ㣨��Χ�У��ཻ������븸�ڵ��ཻ��
	// �ӽڵ㶼�п����ཻ,ֱ��Ҷ�ӽڵ㣬��ʱӦ���ж��Ƿ�������ཻ

	// ���ж��ཻ,���ж������ֽڵ�
	if (node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
		// Ҷ�ӽڵ�
		if (node->left == nullptr && node->right == nullptr)
			return node->object->getIntersection(ray);
	// ���ڵ�
		else
		{
			Intersection l = getIntersection(node->left, ray);
			Intersection r = getIntersection(node->right, ray);

			// �������ǳ��������ཻ��Ϣ
			return l.distance < r.distance ? l : r;
		}
	else
		return interNull;
}