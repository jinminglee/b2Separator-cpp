

# [b2Separator-cpp](https://github.com/mcourteaux/b2Separator-cpp)

Create non-convex, complex shapes with Box2D. Ported from Antoan Angelov's b2Separator class.
This project is forked from [delorenj/b2Separator-cpp](https://github.com/delorenj/b2Separator-cpp), but the C++
was horrible and full of memory leaks. This is a much cleaner and better performing port.

* Source: [https://github.com/mcourteaux/b2Separator-cpp](https://github.com/mcourteaux/b2Separator-cpp)

## Example Use

        b2Body *body;
        b2World *world = ...;
        b2BodyDef bodyDef;
        b2FixtureDef fixtureDef;
        b2Separator sep;

        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(x, y);
        body = world->CreateBody(&bodyDef);
        
        fixtureDef.restitution = 0.4f;
        fixtureDef.friction = 0.2f;
        fixtureDef.density = 4;
        
        std::vector<b2Vec2> vec;
        vec.push_back(b2Vec2(-3, -3));
        vec.push_back(b2Vec2(3, -3));
        vec.push_back(b2Vec2(3, 0));
        vec.push_back(b2Vec2(0, 0));
        vec.push_back(b2Vec2(-3, 3));

        if (sep->Validate(vec) == 0)
        {
            CCLog("Yay! Those vertices are good to go!");
        }
        else
        {
            CCLog("Oh, I guess you effed something up :(");
        }
    
        sep.Separate(body, &fixtureDef, vec);
## Note

You might want to modify the `EPSILON` macro in `b2Seperator.cpp` to suit
your needs. But the default value will be fine in most cases.
