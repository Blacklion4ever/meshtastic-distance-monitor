#include "DistanceMonitorModule.h"
#include "configuration.h"
#include "NodeDB.h"

DistanceMonitorModule::DistanceMonitorModule()
    : SinglePortModule(
          "distance_monitor",
          meshtastic_PortNum_PRIVATE_APP),
      concurrency::OSThread("DistanceMonitor")
{
}

void DistanceMonitorModule::setup()
{
    LOG_INFO(
        "Distance Monitor initialized: role=%s firmware=%s protocol=%d",
        roleName(),
        DISTANCE_MONITOR_FIRMWARE_VERSION,
        DISTANCE_MONITOR_PROTOCOL_VERSION);

    setIntervalFromNow(1000);
}

int32_t DistanceMonitorModule::runOnce()
{
    LOG_DEBUG(
        "Distance Monitor tick: role=%s protocol=%d",
        roleName(),
        DISTANCE_MONITOR_PROTOCOL_VERSION);

    // 1. Trouver la position locale

    if (nodeDB == nullptr)
    {
        LOG_WARN("Local position unavailable: NodeDB not initialized");
        return DISTANCE_MONITOR_TICK_INTERVAL_MS;
    }

    meshtastic_NodeInfoLite *localNode = nodeDB->getMeshNode(nodeDB->getNodeNum());

    if (localNode == nullptr)
    {
        LOG_WARN("Local position unavailable: local node not found");
        return DISTANCE_MONITOR_TICK_INTERVAL_MS;
    }
    // 2. Trouver le nœud cible dans la base Meshtastic
    meshtastic_NodeInfoLite *distNode = nodeDB->getMeshNode(getDistNodeNum());
    if (distNode == nullptr)
    {
        LOG_WARN("Target node unavailable: target node not found");
        return DISTANCE_MONITOR_TICK_INTERVAL_MS;
    }

    if (!nodeDB->hasValidPosition(localNode))
    {
        LOG_INFO("Local position unavailable: waiting for valid GPS position");
        return DISTANCE_MONITOR_TICK_INTERVAL_MS;
    }

    const int32_t latitudeI = localNode->position.latitude_i;
    const int32_t longitudeI = localNode->position.longitude_i;

    const double latitude = static_cast<double>(latitudeI) * 1e-7;
    const double longitude = static_cast<double>(longitudeI) * 1e-7;

    LOG_INFO(
        "Local position available: lat_i=%ld lon_i=%ld lat=%.7f lon=%.7f",
        static_cast<long>(latitudeI),
        static_cast<long>(longitudeI),
        latitude,
        longitude);

    // 3. Vérifier la présence de sa position
    // 4. Vérifier l’âge des données
    // 5. Calculer la distance
    // 6. Afficher l’état
    // 7. Programmer le prochain passage

    return DISTANCE_MONITOR_TICK_INTERVAL_MS;
}

const char *DistanceMonitorModule::roleName()
{
#if DISTANCE_MONITOR_ROLE == 1
    return "base";
#elif DISTANCE_MONITOR_ROLE == 2
    return "tracker";
#else
    return "invalid";
#endif
}

int32_t DistanceMonitorModule::getDistNodeNum()
{
#ifdef DISTANCE_MONITOR_TARGET_NODE
    return DISTANCE_MONITOR_TARGET_NODE;
#else
    return 0;
#endif
}