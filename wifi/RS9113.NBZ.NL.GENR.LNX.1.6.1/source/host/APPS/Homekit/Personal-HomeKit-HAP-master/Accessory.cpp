/*
 * This accessory.cpp is configurated for light accessory
 */

#include "Accessory.h"

#include "PHKAccessory.h"

//Global Level of light strength
int lightStength = 0;
int fanSpeedVal = 0;

void lightIdentify(bool oldValue, bool newValue) {
    printf("Start Identify Light\n");
}

void changeLightPower(bool oldValue, bool newValue) {
    printf("New Light Power State\n");
}

void changeLightIntensity(int oldValue, int newValue) {
    printf("New Intensity\n");
}

#if FAN_ATTRIBUTE == 1
void fanIdentify(bool oldValue, bool newValue) {
    printf("Start Identify Fan\n");
}

void changeFanPower(bool oldValue, bool newValue) {
    printf("New Fan Power State\n");
}

void changeFanDirection(int oldValue, int newValue) {
    printf("New Fan Direction\n");
}

void changeFanSpeed(float oldValue, float newValue) {
    printf("New FAN Speed\n");
}
#endif

#if GDO_ATTRIBUTE == 1
void gdoIdentify(bool oldValue, bool newValue) {
    printf("Start Identify garage door opener\n");
}
#endif

#if OUTLET_ATTRIBUTE == 1
void outletIdentify(bool oldValue, bool newValue) {
    printf("Start Identify outlet\n");
}
#endif

#if SWITCH_ATTRIBUTE == 1
void switchIdentify(bool oldValue, bool newValue) {
    printf("Start Identify switch\n");
}
#endif

#if THERMOSTAT_ATTRIBUTE == 1
void thermostatIdentify(bool oldValue, bool newValue) {
    printf("Start Identify thermostat\n");
}
#endif

#if LOCKMGMT_ATTRIBUTE == 1
void lockmgmtIdentify(bool oldValue, bool newValue) {
    printf("Start Identify lockmgmt\n");
}
#endif

#if LOCKMECH_ATTRIBUTE == 1
void lockmechIdentify(bool oldValue, bool newValue) {
    printf("Start Identify lockmech\n");
}
#endif
#if WINDOW_ATTRIBUTE == 1
void windowIdentify(bool oldValue, bool newValue) {
    printf("Start Identify window\n");
}
#endif

#if WINDOWCVR_ATTRIBUTE == 1
void windowcvrIdentify(bool oldValue, bool newValue) {
    printf("Start Identify window\n");
}
#endif

#if (WINDOW_ATTRIBUTE == 1  || WINDOWCVR_ATTRIBUTE == 1)
void changecurrentposition(bool oldValue, bool newValue) {
    printf("Start changing the position of the dooor\n");
}
#endif

AccessorySet *accSet;

void initAccessorySet() {
    currentDeviceType = deviceType_lightBulb;
    
    printf("Initial Accessory\n");
    //Add Ligth
    accSet = &AccessorySet::getInstance();
    
    Accessory *lightAcc = new Accessory();                                                      //Creting the New Accessory
    addInfoServiceToAccessory(lightAcc, "Light 1", "ET", "Light", "12345678", &lightIdentify);  //Adding service to the accessory
    accSet->addAccessory(lightAcc);                                                             //Adding the accesory the the instance

    Service *lightService = new Service(serviceType_lightBulb);
    lightAcc->addService(lightService);

    stringCharacteristics *lightServiceName = new stringCharacteristics(charType_serviceName, premission_read, 0);
    lightServiceName->setValue("Light");
    lightAcc->addCharacteristics(lightService, lightServiceName);

    boolCharacteristics *powerState = new boolCharacteristics(charType_on, premission_read|premission_write|premission_notify);
    powerState->setValue("true");
    powerState->valueChangeFunctionCall = &changeLightPower;
    lightAcc->addCharacteristics(lightService, powerState);

    intCharacteristics *brightnessState = new intCharacteristics(charType_brightness, premission_read|premission_write|premission_notify, 0, 100, 1, unit_percentage);
    brightnessState->setValue("50");
    brightnessState->valueChangeFunctionCall = &changeLightIntensity;
    lightAcc->addCharacteristics(lightService, brightnessState);

#if FAN_ATTRIBUTE == 1
    /*Fan*/
    Accessory *fan = new Accessory();  
    addInfoServiceToAccessory(fan, "Fan 1", "ET", "Fan", "12345678", &fanIdentify);
    accSet->addAccessory(fan);
   
    Service *fanService = new Service(serviceType_fan);
    fan->addService(fanService);

    stringCharacteristics *fanServiceName = new stringCharacteristics(charType_serviceName, premission_read, 0);
    fanServiceName->setValue("Fan");
    fan->addCharacteristics(fanService, fanServiceName);

    boolCharacteristics *fanPower = new boolCharacteristics(charType_on, premission_read|premission_write|premission_notify);
    fanPower->setValue("true");
    fanPower->valueChangeFunctionCall = &changeFanPower;
    fan->addCharacteristics(fanService, fanPower);
    
    intCharacteristics *directionState = new intCharacteristics(charType_rotationDirection, premission_read|premission_write|premission_notify, 0, 2, 1, unit_none);
    directionState->setValue("2");
    directionState->valueChangeFunctionCall = &changeFanDirection;
    fan->addCharacteristics(fanService, directionState);

    //floatCharacteristics *speedState = new floatCharacteristics(charType_rotationSpeed, premission_read|premission_write|premission_notify, 0, 100, 1, unit_none);
    //speedState->setValue("50");
    //speedState->valueChangeFunctionCall = &changeFanSpeed;
    //fan->addCharacteristics(fanService, speedState);
#endif

#if GDO_ATTRIBUTE == 1
    /*Garage Door Opener*/ 
    Accessory *gdo = new Accessory();
    addInfoServiceToAccessory(gdo, "GDO 1", "ET", "GDO", "12345678", &gdoIdentify);
    accSet->addAccessory(gdo);

    Service *gdoService = new Service(serviceType_garageDoorOpener);
    gdo->addService(gdoService);

    intCharacteristics *currdoorstate = new intCharacteristics(charType_currentDoorState, premission_read|premission_notify,0,5,1,unit_none);
    currdoorstate->setValue("5");
    gdo->addCharacteristics(gdoService, currdoorstate);

    intCharacteristics *tardoorstate = new intCharacteristics(charType_targetDoorState, premission_read|premission_write|premission_notify, 0, 1, 1, unit_none);
    tardoorstate->setValue("1");
    gdo->addCharacteristics(gdoService, tardoorstate);
    
    boolCharacteristics *obstruction = new boolCharacteristics(charType_obstruction, premission_read|premission_notify);
    obstruction->setValue("false");
    //obstruction->valueChangeFunctionCall = &changeFanPower;
    gdo->addCharacteristics(gdoService, obstruction);

    intCharacteristics *lockcurrdoorstate = new intCharacteristics(charType_lockCurrentState, premission_read|premission_notify,0,3,1,unit_none);
    lockcurrdoorstate->setValue("0");
    gdo->addCharacteristics(gdoService, lockcurrdoorstate);

    intCharacteristics *locktardoorstate = new intCharacteristics(charType_lockTargetState , premission_read|premission_write|premission_notify, 0, 1, 1, unit_none);
    locktardoorstate->setValue("1");
    gdo->addCharacteristics(gdoService, locktardoorstate);
 
#endif

#if OUTLET_ATTRIBUTE == 1
    /*Outlet*/
    Accessory *outlet = new Accessory();
    addInfoServiceToAccessory(outlet, "OUTLET 1", "ET", "OUTLET", "12345678", &outletIdentify);
    accSet->addAccessory(outlet);

    Service *outletService = new Service(serviceType_outlet);
    outlet->addService(outletService);

    boolCharacteristics *outletPower = new boolCharacteristics(charType_on, premission_read|premission_write|premission_notify);
    outletPower->setValue("true");
    outlet->addCharacteristics(outletService, outletPower);

    boolCharacteristics *outletinuse = new boolCharacteristics(charType_outletInUse, premission_read|premission_notify);
    outletinuse->setValue("false");
    outlet->addCharacteristics(outletService, outletinuse);
#endif

#if SWITCH_ATTRIBUTE == 1
    /*Switch*/
    Accessory *Switch = new Accessory();
    addInfoServiceToAccessory(Switch, "SWITCH 1", "ET", "SWITCH", "12345678", &switchIdentify);
    accSet->addAccessory(Switch);

    Service *switchService = new Service(serviceType_switch);
    Switch->addService(switchService);

    boolCharacteristics *switchPower = new boolCharacteristics(charType_on, premission_read|premission_write);
    switchPower->setValue("true");
    Switch->addCharacteristics(switchService, switchPower);
#endif

#if LOCKMECH_ATTRIBUTE == 1
    /*Lock Mechanism*/
    Accessory *lockmech = new Accessory();
    addInfoServiceToAccessory(lockmech, "LOCKMECH 1", "ET", "LOCKMECH", "12345678", &lockmechIdentify);
    accSet->addAccessory(lockmech);

    Service *lockmechService = new Service(serviceType_lockMechanism);
    lockmech->addService(lockmechService);

    intCharacteristics *lockcurrstate = new intCharacteristics(charType_lockCurrentState, premission_read|premission_notify, 0, 3, 1, unit_none);
    lockcurrstate->setValue("0");
    lockmech->addCharacteristics(lockmechService, lockcurrstate);

    intCharacteristics *locktarstate = new intCharacteristics(charType_lockTargetState, premission_read|premission_write|premission_notify, 0, 1, 1, unit_none);
    locktarstate->setValue("1");
    lockmech->addCharacteristics(lockmechService, locktarstate);
#endif

#if LOCKMGMT_ATTRIBUTE == 1
    /*Lock Management*/
    char tlv8[3] = {0,1,0};
    Accessory *lockmgmt = new Accessory();
    addInfoServiceToAccessory(lockmgmt, "LOCKMGMT 1", "ET", "LOCKMGMT", "12345678", &lockmgmtIdentify);
    accSet->addAccessory(lockmgmt);

    Service *lockmgmtService = new Service(serviceType_lockManagement);
    lockmgmt->addService(lockmgmtService);

   //!TODO:
    intCharacteristics *lockcontpt = new intCharacteristics(charType_lockControlPoint, premission_write, 0,2, 1, unit_none);
    lockmgmt->addCharacteristics(lockmgmtService, lockcontpt);
#if 0
    intCharacteristics *lockcontpt = new intCharacteristics(charType_lockControlPoint, premission_write, tlv8);
    lockmgmt->addCharacteristics(lockmgmtService, lockcontpt);
#endif

    stringCharacteristics *version = new stringCharacteristics(charType_version, premission_read|premission_notify, 0);
    version->setValue("123456");
    lockmgmt->addCharacteristics(lockmgmtService, version);
#endif

#if WINDOW_ATTRIBUTE == 1
    /*Window*/
    Accessory *window = new Accessory();
    addInfoServiceToAccessory(window, "WINDOW 1", "ET", "WINDOW", "12345678", &windowIdentify);
    accSet->addAccessory(window);

    Service *windowService = new Service(serviceType_window);
    window->addService(windowService);

    //stringCharacteristics *windowServiceName = new stringCharacteristics(charType_serviceName, premission_read, 0);
    //windowServiceName->setValue("WINDOW");
    //window->addCharacteristics(windowService, windowServiceName);

    intCharacteristics *currposition = new intCharacteristics(charType_currentPosition , premission_read|premission_notify, 0, 100, 1, unit_percentage);
    currposition->setValue("0");
    //currposition->valueChangeFunctionCall = &changecurrentposition;
    window->addCharacteristics(windowService, currposition);

    intCharacteristics *tarposition = new intCharacteristics(charType_targetPosition , premission_read|premission_write|premission_notify, 0, 100, 1, unit_percentage);
    tarposition->setValue("100");
    window->addCharacteristics(windowService, tarposition);
    
    intCharacteristics *posState = new intCharacteristics(charType_positionState, premission_read|premission_notify, 0, 2, 1, unit_none );
    posState->setValue("2");
    window->addCharacteristics(windowService,posState);

    //boolCharacteristics *holdposition = new boolCharacteristics(charType_holdPosition, premission_write);
    //holdposition->setValue("true");
    //window->addCharacteristics(windowService, holdposition);
#endif

#if WINDOWCVR_ATTRIBUTE == 1
    /*Window  Covering*/
    Accessory *windowcover = new Accessory();
    addInfoServiceToAccessory(windowcover, "WINDOWCVR 1", "ET", "WINDOWCVR", "12345678", &windowcvrIdentify);
    accSet->addAccessory(windowcover);

    Service *windowcvrService = new Service(serviceType_windowCover);
    windowcover->addService(windowcvrService);

    stringCharacteristics *windowcvrServiceName = new stringCharacteristics(charType_serviceName, premission_read, 0);
    windowcvrServiceName->setValue("WINDOWCOVER");
    windowcover->addCharacteristics(windowcvrService, windowcvrServiceName);

    intCharacteristics *tarposition1 = new intCharacteristics(charType_targetPosition , premission_read|premission_write|premission_notify, 0, 100, 1, unit_percentage);
    tarposition1->setValue("0");
    windowcover->addCharacteristics(windowcvrService, tarposition1);

    intCharacteristics *currposition1 = new intCharacteristics(charType_currentPosition , premission_read|premission_notify, 0, 100, 1, unit_percentage);
    currposition1->setValue("100");
    //currposition->valueChangeFunctionCall = &changecurrentposition;
    windowcover->addCharacteristics(windowcvrService, currposition1);

    
    intCharacteristics *posState1 = new intCharacteristics(charType_positionState, premission_read|premission_notify, 0, 2, 1, unit_none );
    posState1->setValue("2");
    windowcover->addCharacteristics(windowcvrService,posState1);

   boolCharacteristics *holdposition1 = new boolCharacteristics(charType_holdPosition, premission_write);
   holdposition1->setValue("true");
   windowcover->addCharacteristics(windowcvrService, holdposition1);
#endif

#if THERMOSTAT_ATTRIBUTE == 1
    /*Thermostat*/
    Accessory *thermostat = new Accessory();
    addInfoServiceToAccessory(thermostat, "THERMOSTAT 1", "ET", "THERMOSTAT", "12345678", &thermostatIdentify);
    accSet->addAccessory(thermostat);

    Service *thermostatService = new Service(serviceType_thermostat);
    thermostat->addService(thermostatService);

    intCharacteristics *currheatcoolstate = new intCharacteristics(charType_currentHeatCoolMode, premission_read|premission_notify, 0, 2, 1, unit_none);
    currheatcoolstate->setValue("2");
    thermostat->addCharacteristics(thermostatService, currheatcoolstate);

    intCharacteristics *tarheatcoolstate = new intCharacteristics(charType_targetHeatCoolMode, premission_read|premission_write|premission_notify, 0, 3, 1, unit_none);
    tarheatcoolstate->setValue("1");
    thermostat->addCharacteristics(thermostatService, tarheatcoolstate);

    floatCharacteristics *currtemp = new floatCharacteristics(charType_currentTemperature, premission_read|premission_notify, 0, 100, 0.1, unit_celsius);
    currtemp->setValue("60.6");
    thermostat->addCharacteristics(thermostatService, currtemp);
    
    floatCharacteristics *tartemp = new floatCharacteristics(charType_targetTemperature, premission_read|premission_write|premission_notify, 10, 38, 0.1, unit_celsius);
    tartemp->setValue("30.6");
    thermostat->addCharacteristics(thermostatService, tartemp);

    intCharacteristics *tempdispunits = new intCharacteristics(charType_temperatureUnit, premission_read|premission_write|premission_notify, 0, 1, 1, unit_none);
    tempdispunits->setValue("0");
    thermostat->addCharacteristics(thermostatService, tempdispunits);
#endif


};
