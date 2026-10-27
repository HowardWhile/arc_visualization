#ifndef ARC_RVIZ_PLUGINS_IMAGE_TRANSPORT_HINTS_PROPERTY_H
#define ARC_RVIZ_PLUGINS_IMAGE_TRANSPORT_HINTS_PROPERTY_H

#include <string>

#include <rviz_common/properties/property.hpp>
#include <rviz_common/properties/editable_enum_property.hpp>

namespace arc_rviz_plugins {

class ImageTransportHintsProperty : public rviz_common::properties::EditableEnumProperty
{
  Q_OBJECT
 public:
  ImageTransportHintsProperty(const char* name, const char* description,
                              rviz_common::properties::Property* parent, const char* changed_slot);
  ~ImageTransportHintsProperty();

  std::string getTransport();
};
}
#endif
