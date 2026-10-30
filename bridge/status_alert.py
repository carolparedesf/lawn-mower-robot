"""Que alerta va en robot/status (sin ROS). La alerta de seguridad manda; un evento solo se muestra mientras no haya otra."""

EVENT_ALERT_TTL_S = 5.0


def pick_alert(safety_alert, event, event_age, ttl=EVENT_ALERT_TTL_S):
    """safety_alert: ultimo /nav/safety_alert ('none' si no hay). event: ultimo /robot/events o None.
    event_age: segundos desde que llego (None si nunca)."""
    if safety_alert and safety_alert != 'none':
        return safety_alert
    if event and event_age is not None and 0.0 <= event_age <= ttl:
        return event
    return 'none'
