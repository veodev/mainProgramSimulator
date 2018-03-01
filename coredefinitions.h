#ifndef COREDEFINITIONS_H
#define COREDEFINITIONS_H

struct TSignalInCoord
{
    long int Distance;
    unsigned int Delay;
    int Ampl;
};

enum AccessLevel
{
    OperatorLevel = 0,
    LabLevel = 1,
    DeveloperLevel = 2
};

enum DevicePermissions
{
    labPermissions = 0,
    ekasuiPermissions = 1,
    factoryPermissions = 2,
    managerPermissions = 3
};

enum Direction
{
    UnknownDirection,
    ForwardDirection = 1,
    BackwardDirection = -1
};

enum DeviceType
{
    Unknown = -1,
    Avicon31Default = 0,
    Avicon31KZ = 1,
    AviconDB = 2,
    AviconDBHS = 3,
    Avicon15 = 4,
    Avicon31Estonia = 5,
};

enum RegistrationType
{
    DefaultRegistration = 0,
    EKASUIRegistration = 1,
    ASUMagistralRegistration = 2,
};

enum SystemSounds
{
    ACNotify = 0
};

enum ModeSetted
{
    calibrationScanMode = 0,
    calibrationHandMode = 1,
    evaluationMode = 2,
    handMode = 3,
    bScanMode = 4,
    mScanMode = 5,
    menuMode = 6,
    pauseMode = 7,
    headScannerMode = 12
};

#endif  // COREDEFINITIONS_H
