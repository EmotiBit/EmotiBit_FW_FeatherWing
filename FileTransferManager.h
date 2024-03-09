/**************************************************************************/
/*!
    @file     FileTransferManager.h

    This is a library to handle file transfer to/from EmotiBit.

    EmotiBit invests time and resources providing this open source code,
    please support EmotiBit and open-source hardware by purchasing
    products from EmotiBit!

    Written by Nitin Nair for EmotiBit.

    BSD license, all text here must be included in any redistribution
*/
/**************************************************************************/
#ifndef FILE_TRANSFER_MANAGER_H
#define FILE_TRANSFER_MANAGER_H
#include <SimpleFTPServer.h>
#include <String.h>

class FileTransferManager
{
    public:
    enum Mode
    {
        NORMAL = 0,
        FILE_TRANSFER
    }_mode;
    
    enum Protocol
    {
        NONE = -1,
        FTP=0,
        USB_OTG,
        length
    }_protocol=Protocol::NONE;

    struct FtpCreds{
        String username = "ftp";  //!< username to use on the FTP client to log-into the server
        String password = "ftp";  //!< password to use on the FTP client to log-into the server
    }_ftpCreds;

    FtpServer ftpServer;

    /*!
    * @brief Function to setup File transfer
    * @return true, is setup was successful
    */
    bool begin();

    // ToDo: Maybe abstract this to setAuth(). This function can then internally set FTP or other protocol credentials in the future.
    /*!
    * @brief Sets the FTP server credentials
    */
    void setFtpAuth(String username, String password);

    /*!
    * @brief Sets the protocol to be used to for file transfer
    * @param protocol Protocol Type
    * @return True if protocol set successfully, else false
    */
    bool setProtocol(Protocol protocol);

    /*!
    * @brief returns current protocol
    * @return protocol
    */
    Protocol getProtocol();

    /*!
    * @brief sets mode
    * @param mode mode to set
    * @return True is mode is set
    */
    bool setMode(Mode mode);

    /*!
    * @brief returns mode
    * @return current mode set in the FileTransferManager
    */
    Mode getMode();

    // ToDo: In the future, this function will grow to handle other types of file transfer protocols
    /*!
    * @brief handles all the functions to enable file transfer
    */
    void handleTransfer();

};
#endif