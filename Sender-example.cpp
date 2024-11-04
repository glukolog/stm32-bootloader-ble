uint8_t SerialReaderModeUpdate = 0;
uint8_t *fmf = NULL;
uint32_t fmfL = 0, fmf_addr = 0;

#define BUF_RX_SIZE     1024
uint8_t buf_tx[128] = {0}, buf_rx[BUF_RX_SIZE] = {0};
uint16_t buf_tx_len = 0, buf_rx_index = 0, rxi = 0, rxlbufi = 0;

#define UPLOAD_SZ 20
#define MAX_RETRY 10

int16_t test_mode = 0;

    connect(this, SIGNAL(SendUploadPkt()), this, SLOT(Timer_com_send()));


void Main_algo::Timer_com_send() {
   if (SerialReaderModeUpdate) {
       if (buf_tx_len) serial.write((char *)buf_tx, buf_tx_len);
       buf_tx_len = 0;
   }
}

void Main_algo::Timer_DMS_cb() {
    if (SerialReaderModeUpdate) {
        //if (buf_tx_len) serial.write((char *)buf_tx, buf_tx_len);
        //buf_tx_len = 0;
        ui->Upload_PB->setValue(fmf_addr * 100 / fmfL);
        return;
    }
    if (rxlbufi != buf_rx_index) rxlbufi = buf_rx_index;
    else {
      if (rxi != buf_rx_index) {
        uint8_t buf[2048];
        uint16_t bufi = buf_rx_index, len = 0;
        //pr(QString().asprintf("got: %d %d [%d]=%02X %02X %02X", rxi, buf_rx_index, len, buf[0], buf[1], buf[2]));

        if (rxi < bufi) {
            while (rxi < bufi && buf_rx[rxi] != 0x55 && buf_rx[rxi] != '#' && buf_rx[rxi] != 'G') rxi++;
            len = bufi - rxi;
            if (len > 6 && buf_rx[rxi] == 0x55 && buf_rx[rxi + 2] < len) len = buf_rx[rxi + 2];
            memcpy(buf, buf_rx + rxi, len);
            rxi += len;
        } else {
            uint16_t lw = BUF_RX_SIZE - rxi;
            memcpy(buf, buf_rx + rxi, lw);
            memcpy(buf + lw, buf_rx, bufi);
            len = lw + bufi;
            rxi = bufi;
        }
        //pr(QString().asprintf("rxi %d ", rxi));
        if (len) switch (buf[0]) {
        case '#' : buf[len] = 0; pr("@" + QString((char *)buf)); break;
        case 'G' : pr("G = Waiting for update firmware..."); break;
        case 0x55 :
	    // Some other code was here...
            break;
        default : QByteArray data; data.append((const char *)buf, len); pr("Data: " + data.toHex(' ') + QString(data)); break;
        }
        //if (buf[0] == '#') { buf[len] = 0; pr("@" + QString((char *)buf)); }
      }
    }
}

void Main_algo::dataRcvd(QByteArray data) {
    char *buf = data.data();
    int len = data.length();
    if (buf_rx_index + len <= BUF_RX_SIZE) {
        memcpy(buf_rx + buf_rx_index, buf, len);
        buf_rx_index += len;
        if (buf_rx_index == BUF_RX_SIZE) buf_rx_index = 0;
    } else {
        uint16_t lw = BUF_RX_SIZE - buf_rx_index;
        memcpy(buf_rx + buf_rx_index, buf, lw);
        memcpy(buf_rx, buf + lw, len - lw);
        buf_rx_index = len - lw;
    }
}

uint8_t CalcPacketCRC(uint8_t * buf) {
    uint8_t crc = 0;
    for(int i = 0 ; i < 19 ; i++) crc ^= buf[i + 1];
    return crc;
}

void Main_algo::SendFirmware() {
    char buf[UPLOAD_SZ] = {0}, in;
    uint8_t status = 1, tries = 0;
    int16_t in_size, nodata = 0;
    //if (!fmfL) return;
    uint32_t fw_crc = 0, addr = 0, *ptr32 = (uint32_t *)(buf);
    QByteArray data;
    uint32_t Kcnt = 0;
    //clock_t tm = clock();

    //qDebug() << "Upload thread started";

    uint32_t n = 0, *ptr_fmf = (uint32_t *)fmf;
    while (n * 4 < fmfL + 4) {
        fw_crc ^= ptr_fmf[n];
        n++;
    }

    SerialReaderModeUpdate = 1;
    rxi = buf_rx_index;
    fmf_addr = 0;
    st("FW Updating...");
    pr("Updating...");
    while(status) {
        if (rxi != buf_rx_index) {
            in = buf_rx[rxi];
            in_size = 1;
            rxi++;
            if (rxi >= BUF_RX_SIZE) rxi = 0;
            if (rxi != buf_rx_index && buf_rx[rxi] == 'K') {
                in_size = 0;
                //qDebug() << "DK: " << addr << " " << (clock() - tm);
            }
            nodata = 0;
        } else { QThread::msleep(1); nodata++; }
        if (status == 2 && nodata > 2000) {
            nodata = 0;
            tries++;
            in = 'T';
            in_size = 1;
            //qDebug() << "Tries: " << tries << " " << addr << " " << (clock() - tm);
        }
        if (tries > MAX_RETRY) status = 0;
        if (in_size > 0) {
            //if (in != 'K') qDebug() << "up: " << in << " " << addr << " " << (clock() - tm);
            switch (in) {
            case 'G' : // wait for start byte
                if (status == 1) status++; else status = 0;
                buf[1] = 0xFF;
                buf[2] = 0x01;
                ptr32[1] = fmfL;
                ptr32[2] = fw_crc;
                buf[0] = CalcPacketCRC((uint8_t *)buf);
                memcpy(buf_tx, buf, UPLOAD_SZ);
                buf_tx_len = UPLOAD_SZ;
                emit SendUploadPkt();
                break;
            case 'T' : buf_tx_len = UPLOAD_SZ; emit SendUploadPkt(); break; // retry
            case 'X' : buf_tx_len = UPLOAD_SZ; emit SendUploadPkt(); break; // bad crc
            case 'K' : // OK next!
                Kcnt++;
                tries = 0;
                buf[1] = (addr >> 16) & 0xFF;
                buf[2] = (addr >> 8) & 0xFF;
                buf[3] = addr & 0xFF;
                memcpy(buf + 4, fmf + addr, UPLOAD_SZ - 4);
                buf[0] = CalcPacketCRC((uint8_t *)buf);
                while (buf_tx_len) QThread::msleep(10);
                memcpy(buf_tx, buf, UPLOAD_SZ);
                buf_tx_len = UPLOAD_SZ;
                emit SendUploadPkt();
                addr += (UPLOAD_SZ - 4);
                fmf_addr = addr; // status bar
                //if (addr >= fmfL) status++; // NAHUYA?
                break;
            case 'E' : pr("Success!"); status = 0; break;//ui->Upload_PB->setValue(100); break;
            case 'F' : pr("Flashing error!"); status = 0; break;
            case 'R' : pr("Sending error!"); status = 0; break;
            default: pr("Wrong answer!" + QString().asprintf(" %02X %c", in, in)); status = 0; break;
            }
            in_size = 0;
            if (status) { int i = 100; while(buf_tx_len && i) { QThread::msleep(10); i--; }}
        }
        if (nodata > 2000) { status = 0; pr("Timeout!"); }
    }
    qDebug() << "Upload thread ended. " << Kcnt;
    SerialReaderModeUpdate = 0;
}
