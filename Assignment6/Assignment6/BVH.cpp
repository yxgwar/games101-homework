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
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds = Union(centroidBounds, objects[i]->getBounds().Centroid());
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
#if 0
        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();
#else
        int bucket_num = 8;
        std::vector<Bounds3> buckets(bucket_num);
        std::vector<int> bucket_count(bucket_num, 0);

        for(auto object: objects)
        {
            int index = bucket_num * centroidBounds.Offset(object->getBounds().Centroid())[dim];
            index = (index == bucket_num ? index - 1 : index);
            buckets[index] = Union(buckets[index], object->getBounds());
            bucket_count[index]++;
        }

        float cost = std::numeric_limits<float>::max();
        int offset = 0;

        for(int i = 1; i < bucket_num; i++)
        {
            Bounds3 b1;
            Bounds3 b2;
            int count1 = 0, count2 = 0;

            for(int j = 0; j < i; j++)
            {
                b1 = Union(b1, buckets[j]);
                count1 += bucket_count[j];
            }
            for(int k = i; k < bucket_num; k++)
            {
                b2 = Union(b2, buckets[k]);
                count2 += bucket_count[k];
            }
            float cost_tmp = (b1.SurfaceArea() * count1 + b2.SurfaceArea() * count2) / bounds.SurfaceArea();
            if(cost_tmp < cost)
            {
                offset = count1;
                cost = cost_tmp;
            }
        }
        auto beginning = objects.begin();
        auto middling = objects.begin() + offset;
        auto ending = objects.end();
#endif
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
    Intersection inter;

    std::array<int, 3> dirIsNeg;
    for(int i = 0; i < 3; i++)
        dirIsNeg[i] = int(ray.direction[i] > 0);

    if(!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
        return inter;
    
    if(node->left == nullptr && node->right == nullptr)
    {
        return node->object->getIntersection(ray);
    }

    auto hit1 = getIntersection(node->left, ray);
    auto hit2 = getIntersection(node->right, ray);

    return hit1.distance > hit2.distance ? hit2 : hit1;
}