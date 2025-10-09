#ifndef PERSEVERENCE_CONTROLLER_H
#define PERSEVERENCE_CONTROLLER_H

#include "chrono_parsers/ChParserURDF.h"
#include "perseverance_locomotion.h"

using namespace chrono::parsers;
using namespace chrono;

class PerseveranceController {

protected:
    PerseveranceLocomotion locomotion;
    ChParserURDF* m_parser;
    
public:
    void Initialize(ChParserURDF* parser) {
        m_parser = parser;
        locomotion.Initialize(parser);
    }

    virtual void Advance(ChFrame<> pose, double dt) = 0;

    virtual bool IsComplete() = 0;
};

#endif