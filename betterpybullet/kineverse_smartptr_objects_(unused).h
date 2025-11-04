typedef std::shared_ptr<btCollisionShape>         Shape_ptr;
typedef std::shared_ptr<btCollisionObject>        Object_ptr;

//typedef std::shared_ptr<btCollisionShape> Shape_ptr;

class Smart_btCollisionObject : public btCollisionObject {
protected:
    Shape_ptr m_smart_collisionShape;
    std::unordered_set<Object_ptr> m_smart_ignoreset;

public:
    virtual void setCollisionShape(Shape_ptr collisionShape) {
        m_smart_collisionShape = collisionShape;
        btCollisionObject::setCollisionShape(m_smart_collisionShape.get());
    }

    virtual Shape_ptr getCollisionShape() {
        return m_smart_collisionShape;
    }

    void setIgnoreCollisionCheck(Object_ptr co, bool ignoreCollisionCheck) {
        if (ignoreCollisionCheck)
            m_smart_ignoreset.insert(co);
        else
            m_smart_ignoreset.erase(co);
        btCollisionObject::setIgnoreCollisionCheck(co.get(), ignoreCollisionCheck);
    }
};


class Smart_btCollisionWorld : public btCollisionWorld {
protected:
    std::unordered_set<Object_ptr> m_smart_collision_objects;
    Dispatcher_ptr m_smart_dispatcher;
    Broadphase_ptr m_smart_broadphase_pair_cache;
    CollisionConfiguration_ptr m_smart_configuration;

public:
    Smart_btCollisionWorld(Dispatcher_ptr dispatcher, Broadphase_ptr broadphase, CollisionConfiguration_ptr configuration) 
    : m_smart_dispatcher(dispatcher)
    , m_smart_broadphase_pair_cache(broadphase)
    , m_smart_configuration(configuration)
    , btCollisionWorld(dispatcher.get(), broadphase.get(), configuration.get()) {}

    void setBroadphase(Broadphase_ptr pairCache)
    {
        m_smart_broadphase_pair_cache = pairCache;
    }

    const Broadphase_ptr getBroadphase() const
    {
        return m_smart_broadphase_pair_cache;
    }

    Broadphase_ptr getBroadphase()
    {
        return m_smart_broadphase_pair_cache;
    }

    Dispatcher_ptr   getDispatcher()
    {
        return m_smart_dispatcher;
    }

    const Dispatcher_ptr getDispatcher() const
    {
        return m_smart_dispatcher;
    }

    std::vector<Object_ptr> get_collision_objects() {
        return std::vector<Object_ptr>(m_smart_collision_objects.begin(), m_smart_collision_objects.end());
    }

    virtual void addCollisionObject(Object_ptr collisionObject, int collisionFilterGroup=btBroadphaseProxy::DefaultFilter, int collisionFilterMask=btBroadphaseProxy::AllFilter) {
        m_smart_collision_objects.insert(collisionObject);
        btCollisionWorld::addCollisionObject(collisionObject.get(), collisionFilterGroup, collisionFilterMask);
    }

    virtual void removeCollisionObject(Object_ptr collisionObject) {
        m_smart_collision_objects.erase(collisionObject);
        btCollisionWorld::removeCollisionObject(collisionObject.get());
    }
};

class Smart_btCompoundShape : public btCompoundShape {
protected:
    std::vector<Shape_ptr> m_smart_children;

public:

    void addChildShape(const btTransform& localTransform, Shape_ptr shape) {
        m_smart_children.push_back(shape);
        btCompoundShape::addChildShape(localTransform, shape.get());
    }

    virtual void removeChildShape(Shape_ptr shape) {
        btCompoundShape::removeChildShape(shape.get());
        std::remove(m_smart_children.begin(), m_smart_children.end(), shape);
    }

    void removeChildShapeByIndex(int childShapeindex) {
        btCompoundShape::removeChildShapeByIndex(childShapeindex);
        m_smart_children.erase(m_smart_children.begin() + childShapeindex);
    }

    Shape_ptr getChildShape(int index) {
        return m_smart_children[index];
    }

    const Shape_ptr getChildShape(int index) const {
        return m_smart_children[index];
    }
};
