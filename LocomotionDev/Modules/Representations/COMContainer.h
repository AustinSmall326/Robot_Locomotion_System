/** @file    COMContainer.h
 *  @brief   Header file to COMContainer class, which stores COM trajectory parameters.
 *  @author  Austin Small.
 */

#ifndef COMContainer_h_DEFINED
#define COMContainer_h_DEFINED

/** @brief		COMContainer class stores COM trajectory parameters.
 */
class COMContainer
{
    public:
        // Constructor.
        COMContainer(void);
    
        // Update fields methods.
        void clear(void);

        // Getter and setter methods.
        void setX0Pos(double argX0Pos);
        double getX0Pos(void);
        void setX0Vel(double argX0Vel);
        double getX0Vel(void);
        void setY0Pos(double argY0Pos);
        double getY0Pos(void);
        void setY0Vel(double argY0Vel);
        double getY0Vel(void);
        void setR(double argR);
        double getR(void);
        void setRWeight(double argRWeight);
        double getRWeight();
        void setTStart(double argTStart);
        double getTStart(void);
        void setTEnd(double argTEnd);
        double getTEnd(void);
        
    private:
        // Fields.
        double x0Pos;
        double x0Vel;
        double y0Pos;
        double y0Vel;
        double r;
        double rWeight;
        double tStart;
        double tEnd;
};

#endif
