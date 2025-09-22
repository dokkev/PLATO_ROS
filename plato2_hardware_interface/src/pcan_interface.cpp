#include "plato2_hardware_interface/pcan_interface.hpp"

#include "rclcpp/rclcpp.hpp"

namespace pcan_interface {

PCANInterface::PCANInterface() {
    
    // Show PCAN Configuration
    print_current_config();
    std::cout << "Initiliazing CAN..." << std::endl;

    // Initialize PCAN
    TPCANStatus status_result;
    status_result = CAN_Initialize(pcan_handle, bitrate);
    

    if (status_result != PCAN_ERROR_OK) {
        std::cout << "Error initializing CAN" << std::endl;
        return;
    }
    std::cout << "...CAN Initialized" << std::endl;

	// Intialize the Circular Buffer
	RingBuf_ctor(&can_rx_buffer, can_rx_buffer_storage, CAN_RX_BUFFER_SIZE);
}

PCANInterface::~PCANInterface() {
    // Shutdown PCAN
    std::cout << "Shutting Down CAN..." << std::endl;
    CAN_Uninitialize(PCAN_NONEBUS);
}   

////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// PUBLIC FUNCTIONS //////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////


TPCANStatus PCANInterface::write_can(TPCANMsg msg) {
	
	TPCANStatus status = CAN_Write(pcan_handle, &msg);
	std::this_thread::sleep_for(std::chrono::microseconds(300));

    return status;
}

TPCANStatus PCANInterface::read_can(){

	TPCANMsg msg;
	TPCANTimestamp timestamp;

	TPCANStatus status = CAN_Read(pcan_handle, &msg, &timestamp);
	if (status != PCAN_ERROR_QRCVEMPTY && read_can_callback_){
		read_can_callback_(msg);
		// print_message(msg);
	}
	return status;
}

void PCANInterface::send_message(const TPCANMsg &msg){

	TPCANStatus status = write_can(msg);
	// print_message(msg);

	if (status != PCAN_ERROR_OK){
		std::cout << "PCANInterface::write_message:: ERROR! Failed to write message to ID: " << std::hex << msg.ID << std::dec << std::endl;
		show_status(status);
	}
}

void PCANInterface::receive_message(){
	
	TPCANStatus status;
	// read at lease one time the queue looking for messages, and if there is a message found read until the buffer is empty
	// if the queue is empty or error occurs, break the loop
	do{
		status = read_can();
		if (status != PCAN_ERROR_OK){
			// std::cout << "PCANInterface::receive_message:: ERROR! Failed to read message!" << std::endl;
			// show_status(status);
			return;
		}

	} while (!(status & PCAN_ERROR_QRCVEMPTY));
}

void PCANInterface::print_message(const TPCANMsg msg){
	std::cout << "ID: 0x" << std::hex << msg.ID << "   " <<  "Length: " << std::dec << (int)msg.LEN << "   ";
    BYTE data[msg.LEN];
    std::cout << "Data: ";
    for (int i = 0; i < msg.LEN; i++){
        data[i] = msg.DATA[i];
        std::cout << " " << std::hex << (int)data[i];
    }
	std::cout << std::endl;
}

   
////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// PRIVATE FUNCTIONS /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////


void PCANInterface::show_status(TPCANStatus status){
	std::cout << "=========================================================================================\n";
	char buffer[MAX_PATH];
	get_formatted_error(status, buffer);
	std::cout << buffer << "\n";
	std::cout << "=========================================================================================\n";
}

void PCANInterface::print_current_config(){
	std::cout << "==========PCAN Configuration==========\n";
	char buffer[MAX_PATH];
	format_channel_name(pcan_handle, buffer, b_fd);
	std::cout << "* PCANHandle: " << buffer << "\n";
	if (b_fd)
		std::cout << "* CAN FD: True\n";
	else
		std::cout << "* CAN FD: False\n";
	convert_bitrate_to_string(bitrate, buffer);
	std::cout << "* Bitrate: " << buffer << "\n";
    std::cout << "======================================\n";
}


void PCANInterface::format_channel_name(TPCANHandle handle, LPSTR buffer, bool b_fd) {
	TPCANDevice devDevice;
	BYTE byChannel;

	// Gets the owner device and channel for a PCAN-Basic handle
	if (handle < 0x100)
	{
		devDevice = (TPCANDevice)(handle >> 4);
		byChannel = (BYTE)(handle & 0xF);
	}
	else
	{
		devDevice = (TPCANDevice)(handle >> 8);
		byChannel = (BYTE)(handle & 0xFF);
	}

	// Constructs the PCAN-Basic Channel name and return it
	char handleBuffer[MAX_PATH];
	get_handle_name(handle, handleBuffer);
	if (b_fd)
		sprintf_s(buffer, MAX_PATH, "%s:FD %d (%Xh)", handleBuffer, byChannel, handle);
	else
		sprintf_s(buffer, MAX_PATH, "%s %d (%Xh)", handleBuffer, byChannel, handle);
}

void PCANInterface::get_handle_name(TPCANHandle handle, LPSTR buffer){
	strcpy_s(buffer, MAX_PATH, "PCAN_NONE");
	switch (handle)
	{
	case PCAN_PCIBUS1:
	case PCAN_PCIBUS2:
	case PCAN_PCIBUS3:
	case PCAN_PCIBUS4:
	case PCAN_PCIBUS5:
	case PCAN_PCIBUS6:
	case PCAN_PCIBUS7:
	case PCAN_PCIBUS8:
	case PCAN_PCIBUS9:
	case PCAN_PCIBUS10:
	case PCAN_PCIBUS11:
	case PCAN_PCIBUS12:
	case PCAN_PCIBUS13:
	case PCAN_PCIBUS14:
	case PCAN_PCIBUS15:
	case PCAN_PCIBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_PCI");
		break;

	case PCAN_USBBUS1:
	case PCAN_USBBUS2:
	case PCAN_USBBUS3:
	case PCAN_USBBUS4:
	case PCAN_USBBUS5:
	case PCAN_USBBUS6:
	case PCAN_USBBUS7:
	case PCAN_USBBUS8:
	case PCAN_USBBUS9:
	case PCAN_USBBUS10:
	case PCAN_USBBUS11:
	case PCAN_USBBUS12:
	case PCAN_USBBUS13:
	case PCAN_USBBUS14:
	case PCAN_USBBUS15:
	case PCAN_USBBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_USB");
		break;

	case PCAN_LANBUS1:
	case PCAN_LANBUS2:
	case PCAN_LANBUS3:
	case PCAN_LANBUS4:
	case PCAN_LANBUS5:
	case PCAN_LANBUS6:
	case PCAN_LANBUS7:
	case PCAN_LANBUS8:
	case PCAN_LANBUS9:
	case PCAN_LANBUS10:
	case PCAN_LANBUS11:
	case PCAN_LANBUS12:
	case PCAN_LANBUS13:
	case PCAN_LANBUS14:
	case PCAN_LANBUS15:
	case PCAN_LANBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_LAN");
		break;

	default:
		strcpy_s(buffer, MAX_PATH, "UNKNOWN");
		break;
	}
}

void PCANInterface::get_formatted_error(TPCANStatus error, LPSTR buffer){
    if (CAN_GetErrorText(error, 0x09, buffer) != PCAN_ERROR_OK)
    sprintf_s(buffer, MAX_PATH, "An error occurred. Error-code's text (%Xh) couldn't be retrieved", error);
}

void PCANInterface::convert_bitrate_to_string(TPCANBaudrate bitrate, LPSTR buffer){
	switch (bitrate)
	{
	case PCAN_BAUD_1M:
		strcpy_s(buffer, MAX_PATH, "1 MBit/sec");
		break;
	case PCAN_BAUD_800K:
		strcpy_s(buffer, MAX_PATH, "800 kBit/sec");
		break;
	case PCAN_BAUD_500K:
		strcpy_s(buffer, MAX_PATH, "500 kBit/sec");
		break;
	case PCAN_BAUD_250K:
		strcpy_s(buffer, MAX_PATH, "250 kBit/sec");
		break;
	case PCAN_BAUD_125K:
		strcpy_s(buffer, MAX_PATH, "125 kBit/sec");
		break;
	case PCAN_BAUD_100K:
		strcpy_s(buffer, MAX_PATH, "100 kBit/sec");
		break;
	case PCAN_BAUD_95K:
		strcpy_s(buffer, MAX_PATH, "95,238 kBit/sec");
		break;
	case PCAN_BAUD_83K:
		strcpy_s(buffer, MAX_PATH, "83,333 kBit/sec");
		break;
	case PCAN_BAUD_50K:
		strcpy_s(buffer, MAX_PATH, "50 kBit/sec");
		break;
	case PCAN_BAUD_47K:
		strcpy_s(buffer, MAX_PATH, "47,619 kBit/sec");
		break;
	case PCAN_BAUD_33K:
		strcpy_s(buffer, MAX_PATH, "33,333 kBit/sec");
		break;
	case PCAN_BAUD_20K:
		strcpy_s(buffer, MAX_PATH, "20 kBit/sec");
		break;
	case PCAN_BAUD_10K:
		strcpy_s(buffer, MAX_PATH, "10 kBit/sec");
		break;
	case PCAN_BAUD_5K:
		strcpy_s(buffer, MAX_PATH, "5 kBit/sec");
		break;
	default:
		strcpy_s(buffer, MAX_PATH, "Unknown Bitrate");
		break;
	}

	}

} // namespace pcan_interface