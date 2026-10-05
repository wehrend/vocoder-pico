# Erzeugt build_stamp.h mit dem aktuellen Zeitpunkt. Wird bei JEDEM Build
# ausgeführt (siehe CMakeLists.txt), damit die Diagnoseausgabe immer zeigt,
# wann der laufende Stand gebaut wurde.
string(TIMESTAMP STAMP "%b %d %Y %H:%M:%S")
set(CONTENT "#pragma once\n// Automatisch erzeugt bei jedem Build - nicht von Hand bearbeiten.\nconstexpr const char kBuildStamp[] = \"${STAMP}\";\n")
# Nur schreiben, wenn sich der Inhalt ändert (er ändert sich jede Sekunde).
if(EXISTS "${OUT}")
    file(READ "${OUT}" OLD)
endif()
if(NOT "${OLD}" STREQUAL "${CONTENT}")
    file(WRITE "${OUT}" "${CONTENT}")
endif()