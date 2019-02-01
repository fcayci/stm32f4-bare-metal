#ifndef __USB_VCP_H
#define __USB_VCP_H

#ifndef   __PACKED
  #define __PACKED           __attribute__((packed, aligned(1)))
#endif

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5740,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const char *usb_strings[] = {
	"furkan.space",
	"STM32 Virtual COM Port",
	"1",
};

/*
 * This notification endpoint isn't implemented. According to CDC spec it's
 * optional, but its absence causes a NULL pointer dereference in the
 * Linux cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {
	{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
	}
};

static const struct usb_endpoint_descriptor data_endp[] = {
	{
	 .bLength = USB_DT_ENDPOINT_SIZE,
	 .bDescriptorType = USB_DT_ENDPOINT,
	 .bEndpointAddress = 0x01,
	 .bmAttributes = USB_ENDPOINT_ATTR_BULK,
	 .wMaxPacketSize = 64,
	 .bInterval = 1,
	 },
	{
	 .bLength = USB_DT_ENDPOINT_SIZE,
	 .bDescriptorType = USB_DT_ENDPOINT,
	 .bEndpointAddress = 0x82,
	 .bmAttributes = USB_ENDPOINT_ATTR_BULK,
	 .wMaxPacketSize = 64,
	 .bInterval = 1,
	 }
};

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __PACKED cdcacm_functional_descriptors = {
	.header =
	/* struct usb_cdc_header_descriptor header */
	{
	 .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
	 .bDescriptorType = CS_INTERFACE,
	 .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
	 .bcdCDC = 0x0110,
	 },
	.call_mgmt =
	/* struct usb_cdc_call_management_descriptor call_mgmt */
	{
	 .bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor),
	 .bDescriptorType = CS_INTERFACE,
	 .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
	 .bmCapabilities = 0,
	 .bDataInterface = 1,
	 },
	.acm =
	/* struct usb_cdc_acm_descriptor acm */
	{
	 .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
	 .bDescriptorType = CS_INTERFACE,
	 .bDescriptorSubtype = USB_CDC_TYPE_ACM,
	 .bmCapabilities = 0,
	 },
	.cdc_union =
	/* struct usb_cdc_union_descriptor cdc_union */
	{
	 .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
	 .bDescriptorType = CS_INTERFACE,
	 .bDescriptorSubtype = USB_CDC_TYPE_UNION,
	 .bControlInterface = 0,
	 .bSubordinateInterface0 = 1,
	 }
};

static const struct usb_interface_descriptor comm_iface[] = {
	{
	 .bLength = USB_DT_INTERFACE_SIZE,
	 .bDescriptorType = USB_DT_INTERFACE,
	 .bInterfaceNumber = 0,
	 .bAlternateSetting = 0,
	 .bNumEndpoints = 1,
	 .bInterfaceClass = USB_CLASS_CDC,
	 .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	 .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	 .iInterface = 0,
	 .endpoint = comm_endp,
	 .extra = &cdcacm_functional_descriptors,
	 .extralen = sizeof(cdcacm_functional_descriptors)
	 }
};

static const struct usb_interface_descriptor data_iface[] = {
	{
	 .bLength = USB_DT_INTERFACE_SIZE,
	 .bDescriptorType = USB_DT_INTERFACE,
	 .bInterfaceNumber = 1,
	 .bAlternateSetting = 0,
	 .bNumEndpoints = 2,
	 .bInterfaceClass = USB_CLASS_DATA,
	 .bInterfaceSubClass = 0,
	 .bInterfaceProtocol = 0,
	 .iInterface = 0,
	 .endpoint = data_endp,
	 }
};

static const struct usb_interface ifaces[] = {
	{
	 .num_altsetting = 1,
	 .altsetting = comm_iface,
	 },
	{
	 .num_altsetting = 1,
	 .altsetting = data_iface,
	 }
};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,
	.interface = ifaces,
};

static usbd_device *setup_usb(uint8_t *usbd_control_buffer, size_t buf_len);
static enum usbd_request_return_codes cdcacm_control_request(
	usbd_device *usbd_dev,
	struct usb_setup_data *req,
	uint8_t **buf,
	uint16_t *len,
	void (**complete)(
		usbd_device *usbd_dev,
		struct usb_setup_data *req)
	);
static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue);
static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep);

#endif