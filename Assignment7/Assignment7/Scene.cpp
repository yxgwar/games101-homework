//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
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
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f L_dir, L_indir;

    Intersection inter = intersect(ray);
    if(inter.happened)
    {
        if(inter.m->hasEmission()) return inter.m->getEmission();
        Vector3f p = inter.coords;
        Vector3f wo = ray.direction;
        Vector3f n = inter.normal;

        Intersection p_inter;
        float pdf;
        sampleLight(p_inter, pdf);
        Vector3f x = p_inter.coords;
        Vector3f ws = normalize(x - p);
        Vector3f NN = p_inter.normal;
        Vector3f emit = p_inter.emit;

        Intersection x_inter = intersect(Ray(p, ws));
        float dir = (x - p).norm();
        if(x_inter.distance - dir > -0.001f)
            L_dir = emit * inter.m->eval(wo, ws, n) * dotProduct(ws, n) * dotProduct(-ws, NN) / (dir * dir * pdf);
        
        if(get_random_float() < RussianRoulette)
        {
            Vector3f wi = inter.m->sample(wo, n);
            Intersection r_inter = intersect(Ray(p, wi));
            if(r_inter.happened && !r_inter.m->hasEmission())
            {
                pdf = inter.m->pdf(ray.direction, wi, n);
                if (pdf > 0.001f)
                    L_indir = castRay(Ray(p, wi), depth + 1) * inter.m->eval(wo, wi, n) * dotProduct(wi, n) / (pdf * RussianRoulette);
            }
        }
    }
    return L_dir + L_indir;
}