#include "driver.h"


const float Driver::MAX_UNSTUCK_ANGLE = 30.0/180.0*PI; /* [radians] */
const float Driver::UNSTUCK_TIME_LIMIT = 2.0; /* [s] */
const float Driver::MAX_UNSTUCK_SPEED = 5.0; /* [m/s] */
const float Driver::MIN_UNSTUCK_DIST = 3.0; /* [m] */
const float Driver::G = 9.81; /* [m/(s*s)] */
const float Driver::FULL_ACCEL_MARGIN = 1.0; /* [m/s] */
const float Driver::LOOKAHEAD_CONST = 17.0; /* [m] */
const float Driver::LOOKAHEAD_FACTOR = 0.33; /* [1/s] */
const float Driver::WIDTHDIV = 4.0; /* [-] */


Driver::Driver(int index)
{
	INDEX = index;
}

/* called for every track change or new race */
void Driver::initTrack(tTrack* t, void *carHandle,
				void **carParmHandle, tSituation *s)
{
	track = t;
	*carParmHandle = NULL;
}


/* Start a new race */
void Driver::newRace(tCarElt* car, tSituation *s)
{
	MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT/RCM_MAX_DT_ROBOTS);
	stuck = 0;
}


/* Drive during race */
void Driver::drive(tCarElt* car, tSituation *s)
{
	update(car, s);

	memset(&car->ctrl, 0, sizeof(tCarCtrl));

	if(isStuck(car)){
		car->ctrl.steer = -angle / car->_steerLock;
		car->ctrl.gear = -1; // reverse gear
		car->ctrl.accelCmd = 0.5; // 50% accelerator pedal
		car->ctrl.brakeCmd = 0.0; // no brakes	
	} else {
		car->ctrl.steer = getSteer(car);
		car->ctrl.gear = 4;
		car->ctrl.brakeCmd = getBrake(car);
		if (car->ctrl.brakeCmd == 0.0){
			car->ctrl.accelCmd = filterTrk(getAccel(car), car);
		} else {
			car->ctrl.accelCmd = 0.0;
		}
	}
}


/* Set pitstop command */
int Driver::pitCommand(tCarElt* car, tSituation *s)
{
	return ROB_PIT_IM; /* return immediately */
}


/* End od Current race */
void Driver::endRace(tCarElt *car, tSituation *s)
{
}


/* Update my private data every timestep */
void Driver::update(tCarElt* car, tSituation *s)
{
	trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
	angle = trackangle - car->_yaw;
	NORM_PI_PI(angle);
}


/* Check if I'm stuck */
bool Driver::isStuck(tCarElt* car)
{
	if (fabs(angle) < MAX_UNSTUCK_ANGLE && 
		car->_speed_x < MAX_UNSTUCK_SPEED && 
		fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST) {
		if (stuck > MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle*angle < 0.0){
			return true;	
		} else {
			stuck++;
			return false;
		}
	} else {
		stuck = 0;
		return false;	
	}
}

/* Compute the the allowed speed on the segment*/
float Driver::getAllowedSpeed(tTrackSeg *segment)
{
	if(segment->type == TR_STR){
		return FLT_MAX;
	} else {
		float arc = 0.0;
		tTrackSeg *s = segment;

		while(s->type == segment->type && arc < PI/2.0){
			arc += s->arc;
			s = s->next;	
		}
		arc /= PI/2.0;
		float mu = segment->surface->kFriction;
		float r = (segment->radius + segment->width/2.0) / sqrt(arc);
		return sqrt((mu*G*r)/(1.0 - MIN(1.0, r*CA*mu/mass)));
	}
}


/* Compute the length to the end of the segment */
float Driver::getDistToSegEnd(tCarElt* car)
{
	if(car->_trkPos.seg->type == TR_STR){
		return car->_trkPos.seg->length - car->_trkPos.toStart;	
	} else {
		return (car->_trkPos.seg->arc - car->_trkPos.toStart)*car->_trkPos.seg->radius;
	}
}



/* Compute fitting acceleration */
float Driver::getAccel(tCarElt* car)
{
	float allowedspeed = getAllowedSpeed(car->_trkPos.seg);
	float gr = car->_gearRatio[car->_gear + car->_gearOffset];
	float rm = car->_enginerpmRedLine;
	if (allowedspeed > car->_speed_x + FULL_ACCEL_MARGIN){
		return 1.0;
	} else {
		return allowedspeed/car->_wheelRadius(REAR_RGT)*gr/ rm;
	}
}


float Driver::getBrake(tCarElt* car)
{
	tTrackSeg *segptr = car->_trkPos.seg;
	float currentspeedsqr = car->_speed_x*car->_speed_x;
	float mu = segptr->surface->kFriction;
	float maxlookaheaddist = currentspeedsqr / (2.0*mu*G);
	float lookaheaddist = getDistToSegEnd(car);
	float allowedspeed = getAllowedSpeed(segptr);
	if(allowedspeed < car->_speed_x) return 1.0;
	segptr = segptr->next;
	while( lookaheaddist < maxlookaheaddist){
		allowedspeed = getAllowedSpeed(segptr);
		if(allowedspeed < car->_speed_x){
			float allowedspeedsqr = allowedspeed*allowedspeed;
			float brakedist = (currentspeedsqr = allowedspeedsqr) / (2.0*mu*G);
			if (brakedist > lookaheaddist){
				return 1.0; // apply full brake
			}
		}
		lookaheaddist += segptr->length;
		segptr = segptr->next;
	}
	return 0.0;
}



v2d Driver::getTargetPoint(tCarElt* car){
	tTrackSeg *seg = car->_trkPos.seg;
	float lookahead = LOOKAHEAD_CONST + car->_speed_x*LOOKAHEAD_FACTOR;
	float length = getDistToSegEnd(car);

	while(length < lookahead) {
		seg = seg->next;
		length += seg->length;
	}
	length = lookahead = length + seg->length;
	v2d s;
	s.x = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x)/2.0;
	s.y = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y)/2.0;
	if (seg->type == TR_STR) {
		v2d d;  
		d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x)/seg->length;		
		d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y)/seg->length;		
		return s + d*length;
	} else {
		v2d c;
		c.x = seg->center.x;
		c.y = seg->center.y;
		float arc = length /seg->radius;
		float arcsign = (seg->type == TR_RGT) ? -1 : 1; 
		arc = arc*arcsign;
		return s.rotate(c, arc);
	}
}



float Driver::getSteer(tCarElt* car){
	float targetAngle; 
	v2d target = getTargetPoint(car);

	targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
	targetAngle -= car->_yaw;
	NORM_PI_PI(targetAngle);
	return targetAngle / car->_steerLock;
}


/* Hold the car on the track */
float Driver::filterTrk(float accel, tCarElt* car)
{
	tTrackSeg* seg = car->_trkPos.seg;

	if (car->_speed_x < MAX_UNSTUCK_SPEED) return accel;

	if (seg->type == TR_STR) {
		float tm  = fabs(car->_trkPos.toMiddle);
		float w = seg->width/WIDTHDIV;
		if  (tm > w) return 0.0; else return accel;
	} else {
		float sign = (seg->type == TR_RGT) ? -1 : 1;	
		if (car->_trkPos.toMiddle * sign > 0.0){
			return accel;
		} else {
			float tm = fabs(car->_trkPos.toMiddle);
			float w = seg->width/WIDTHDIV;
			if (tm > w) return 0.0; else return accel;
		}
	}
}

