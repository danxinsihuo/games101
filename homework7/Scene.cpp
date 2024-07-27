//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH() {
	printf(" - Generating BVH...\n\n");
	this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray& ray) const
{
	return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection& pos, float& pdf) const
{
	float emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
		}
	}
	float p = get_random_float() * emit_area_sum;
	emit_area_sum = 0;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		if (objects[k]->hasEmit()) {
			emit_area_sum += objects[k]->getArea();
			if (p <= emit_area_sum) {
				objects[k]->Sample(pos, pdf);
				break;
			}
		}
	}
}

bool Scene::trace(
	const Ray& ray,
	const std::vector<Object*>& objects,
	float& tNear, uint32_t& index, Object** hitObject)
{
	*hitObject = nullptr;
	for (uint32_t k = 0; k < objects.size(); ++k) {
		float tNearK = kInfinity;
		uint32_t indexK;
		Vector2f uvK;
		if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
			*hitObject = objects[k];
			tNear = tNearK;
			index = indexK;
		}
	}
	return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray& ray, int depth) const
{
	// TO DO Implement Path Tracing Algorithm here
	Vector3f L_dir;
	Vector3f L_indir;

	Intersection inter = intersect(ray);
	// 不相交，直接返回
	if (!inter.happened) {
		return L_dir;
	}

	if (inter.m->hasEmission()) {
		return inter.m->getEmission();
	}
	Vector3f p = inter.coords;
	Material* m = inter.m;
	Vector3f N = inter.normal.normalized();
	Vector3f wo = ray.direction;

	float pdf_Light;
	Intersection sample_point;

	sampleLight(sample_point, pdf_Light);

	Vector3f x = sample_point.coords;
	Vector3f ws = (x - p).normalized();
	Vector3f NN = sample_point.normal.normalized();
	Vector3f emit = sample_point.emit;
	float d = (x - p).norm();

	Ray Obj2Light(p, ws);
	float d2 = intersect(Obj2Light).distance;
	if (d - d2 < 0.001) {
		L_dir = emit * m->eval(wo, ws, N) * dotProduct(N, ws) * dotProduct(NN, -ws) / std::pow(d, 2) / pdf_Light;
	}

	if (get_random_float() < RussianRoulette) {
		Vector3f wi = m->sample(wo, N).normalized();
		Ray r(p, wi);
		Intersection inter = intersect(r);
		if (inter.happened && !inter.m->hasEmission()) {
			L_indir = castRay(r, depth + 1) * m->eval(wo, wi, N) * dotProduct(wi, N) / m->pdf(wo, wi, N) / RussianRoulette;
		}
	}
	return L_dir + L_indir;
}