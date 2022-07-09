//! # I2C Peripheral (slave) implementation
//!
//! The RP2040 I2C block can behave as a peripheral node on an I2C bus.
//!
//! In order to handle peripheral transactions this driver exposes an iterator streaming I2C event
//! that the usercode must handle to properly handle the I2C communitation. See [`I2CEvent`] for a
//! list of events to handle.
//!
//! Although [`Start`](I2CEvent::Start), [`Restart`](I2CEvent::Restart) and [`Stop`](I2CEvent::Stop)
//! events may not require any action on the device, [`TransferRead`](I2CEvent::TransferRead) and
//! [`TransferWrite`](I2CEvent::TransferWrite) require some action:
//!
//! - [`TransferRead`](I2CEvent::TransferRead): A controller is attempting to read from this peripheral.  
//!   The I2C block holds the SCL line low (clock stretching) until data is pushed to the transmission
//!   FIFO by the user application using [`write`](I2CPeripheralEventIterator::write).  
//!   Data remaining in the FIFO when the bus constroller stops the transfer are ignored & the fifo
//!   is flushed.
//! - [`TransferWrite`](I2CEvent::TransferWrite): A controller is sending data to this peripheral.  
//!   The I2C block holds the SCL line (clock stretching) until there is room for more data in the
//!   Rx FIFO using [`read`](I2CPeripheralEventIterator::read).  
//!   Data are automatically acknowledged by the I2C block and it is not possible to NACK incoming
//!   data comming to the rp2040.

use core::{marker::PhantomData, ops::Deref};

use super::{Peripheral, ValidPinScl, ValidPinSda, I2C};
use crate::{
    pac::{i2c0::RegisterBlock as I2CBlock, RESETS},
    resets::SubsystemReset,
};

/// I2C bus events
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum I2CEvent {
    /// Start condition has been detected.
    Start,
    /// Restart condition has been detected.
    Restart,
    /// The controller requests data.
    TransferRead,
    /// The controller sends data.
    TransferWrite,
    /// Stop condition detected.
    Stop,
}

#[derive(Debug, Clone, Copy)]
enum State {
    Idle,
    Active,
    Read,
    Write,
}

/// Provides Async features to I2C peripheral.
pub struct I2CPeripheralEventIterator<Block, Pins> {
    i2c: I2C<Block, Pins, Peripheral>,
    state: State,
}

/// I2C Interrupts.
pub enum I2CPeripheralIrqs {
    /// Start or restart condition has been detected.
    StartDet,
    /// Set only when a General Call address is received and it is acknowledged
    GenCall,
    /// Controller is attempting to read from this peripheral.
    RdReq,
    /// Restart condition has been detected.
    RestartDet,
    /// Read transfer not accepted by controller end. Should happen on last byte of transfers.
    RxDone,
    /// Triggered when the receive buffer reaches or goes above the RX_TL threshold in the IC_RX_TL
    /// register. It is automatically cleared by hardware when buffer level goes below the
    /// threshold. If the module is disabled (IC_ENABLE[0]=0), the RX FIFO is flushed and held in
    /// reset; therefore the RX FIFO is not full. So this interrupt is cleared once the
    /// IC_ENABLE bit 0 is programmed with a 0, regardless of the activity that continues.
    RxFull,
    /// Set if the receive buffer is completely filled to IC_RX_BUFFER_DEPTH and an additional byte
    /// is received from an external I2C device. The DW_apb_i2c acknowledges this, but any data
    /// bytes received after the FIFO is full are lost. If the module is disabled (IC_ENABLE[0]=0),
    /// this interrupt stays triggered until the controller or peripheral state machines go into
    /// idle, and when ic_en goes to 0, this interrupt is cleared.
    /// Note:  If bit 9 of the IC_CON register (RX_FIFO_FULL_HLD_CTRL) is programmed to HIGH, then
    /// the RxOver interrupt never occurs, because the Rx FIFO never overflows.
    RxOver,
    /// Set if the processor attempts to read the receive buffer when it is empty by reading from
    /// the IC_DATA_CMD register. If the module is disabled (IC_ENABLE[0]=0), this interrupt stays
    /// triggered until the controller or peripheral state machines go into idle, and when ic_en
    /// goes to 0, this interrupt is cleared.
    RxUnder,
    /// Stop condition has been detected.
    StopDet,
    /// Unable to complete the transfer. the IC_TX_ABRT_SOURCE register indicates the reason why the
    /// transmit abort takes places.
    TxAbrt,
    /// The behavior of the TxEmpty interrupt status differs based on the TX_EMPTY_CTRL selection in
    /// the IC_CON register.
    /// * When TX_EMPTY_CTRL = 0: This interrupt is triggered when the transmit buffer is at or
    ///   below the threshold value set in the IC_TX_TL register.
    /// * When TX_EMPTY_CTRL = 1: This interrupt is triggered when the transmit buffer is at or
    ///   below the threshold value set in the IC_TX_TL register and the transmission of the
    ///   address/data from the internal shift register for the most recently popped command is
    ///   completed. It is automatically cleared by hardware when the buffer level goes above the
    ///   threshold. When IC_ENABLE[0] is set to 0, the TX FIFO is flushed and held in reset. There
    ///   the TX FIFO looks like it has no data within it, so this interrupt is triggered, provided
    ///   there is activity in the controller or peripheral state machines. When there is no longer
    ///   any activity, then with ic_en=0, this interrupt is cleared.
    TxEmpty,
    /// Set during transmit if the transmit buffer is filled to IC_TX_BUFFER_DEPTH and the processor
    /// attempts to issue another I2C command by writing to the IC_DATA_CMD register. When the
    /// module is disabled, this interrupt stays triggered until the controller or peripheral state
    /// machines go into idle, and when ic_en goes to 0, this interrupt is cleared.
    TxOver,
    /// Activity on the I2C bus has been detected.
    Activity,
}

impl<T, Sda, Scl> I2C<T, (Sda, Scl), Peripheral>
where
    T: SubsystemReset + Deref<Target = I2CBlock>,
    Sda: ValidPinSda<T>,
    Scl: ValidPinScl<T>,
{
    /// Configures the I2C peripheral to work in peripheral mode
    ///
    /// The bus *MUST* be idle when this method is called.
    #[inline]
    #[allow(clippy::type_complexity)]
    pub fn new_peripheral_event_iterator(
        i2c: T,
        sda_pin: Sda,
        scl_pin: Scl,
        resets: &mut RESETS,
        addr: u16,
    ) -> I2CPeripheralEventIterator<T, (Sda, Scl)> {
        Self::new_peripheral_event_iterator_with_irq(i2c, sda_pin, scl_pin, resets, addr, &[])
    }

    /// Configures the I2C peripheral to work in peripheral mode
    ///
    /// The bus *MUST* be idle when this method is called.
    pub fn new_peripheral_event_iterator_with_irq(
        i2c: T,
        sda_pin: Sda,
        scl_pin: Scl,
        resets: &mut RESETS,
        addr: u16,
        irqs: &[I2CPeripheralIrqs],
    ) -> I2CPeripheralEventIterator<T, (Sda, Scl)> {
        i2c.reset_bring_down(resets);
        i2c.reset_bring_up(resets);

        i2c.ic_enable.write(|w| w.enable().disabled());

        // TODO: rp2040 supports 10bits addressing
        //i2c_reserved_addr(addr)
        i2c.ic_sar.write(|w| unsafe { w.ic_sar().bits(addr) });
        // select peripheral mode & speed
        i2c.ic_con.modify(|_, w| {
            // run in fast mode
            w.speed().fast();
            // setup slave mode
            w.master_mode().disabled();
            w.ic_slave_disable().slave_enabled();
            // hold scl when fifo's full
            w.rx_fifo_full_hld_ctrl().enabled();
            w.ic_restart_en().enabled();
            w.tx_empty_ctrl().enabled()
        });

        // Clear FIFO threshold
        i2c.ic_tx_tl.write(|w| unsafe { w.tx_tl().bits(0) });
        i2c.ic_rx_tl.write(|w| unsafe { w.rx_tl().bits(0) });

        if !irqs.is_empty() {
            i2c.ic_intr_mask.write(|w| {
                // Mask all interrupts initially.
                w.m_start_det().enabled();
                w.m_gen_call().enabled();
                w.m_rd_req().enabled();
                w.m_restart_det().enabled();
                w.m_rx_done().enabled();
                w.m_rx_full().enabled();
                w.m_rx_over().enabled();
                w.m_rx_under().enabled();
                w.m_stop_det().enabled();
                w.m_tx_abrt().enabled();
                w.m_tx_empty().enabled();
                w.m_tx_over().enabled();
                w.m_activity().enabled();

                // Unmask requested interrupts.
                for irq in irqs {
                    match irq {
                        I2CPeripheralIrqs::StartDet => {
                            w.m_start_det().disabled();
                        }
                        I2CPeripheralIrqs::GenCall => {
                            w.m_gen_call().disabled();
                        }
                        I2CPeripheralIrqs::RdReq => {
                            w.m_rd_req().disabled();
                        }
                        I2CPeripheralIrqs::RestartDet => {
                            w.m_restart_det().disabled();
                        }
                        I2CPeripheralIrqs::RxDone => {
                            w.m_rx_done().disabled();
                        }
                        I2CPeripheralIrqs::RxFull => {
                            w.m_rx_full().disabled();
                        }
                        I2CPeripheralIrqs::RxOver => {
                            w.m_rx_over().disabled();
                        }
                        I2CPeripheralIrqs::RxUnder => {
                            w.m_rx_under().disabled();
                        }
                        I2CPeripheralIrqs::StopDet => {
                            w.m_stop_det().disabled();
                        }
                        I2CPeripheralIrqs::TxAbrt => {
                            w.m_tx_abrt().disabled();
                        }
                        I2CPeripheralIrqs::TxEmpty => {
                            w.m_tx_empty().disabled();
                        }
                        I2CPeripheralIrqs::TxOver => {
                            w.m_tx_over().disabled();
                        }
                        I2CPeripheralIrqs::Activity => {
                            w.m_activity().disabled();
                        }
                    }
                }
                w
            });
        }

        // Enable I2C block
        i2c.ic_enable.write(|w| w.enable().enabled());

        I2CPeripheralEventIterator {
            i2c: Self {
                i2c,
                pins: (sda_pin, scl_pin),
                mode: PhantomData,
            },
            state: State::Idle,
        }
    }
}

impl<T: Deref<Target = I2CBlock>, PINS> I2CPeripheralEventIterator<T, PINS> {
    /// Push up to `usize::min(TX_FIFO_SIZE, buf.len())` bytes to the TX FIFO.
    /// Returns the number of bytes pushed to the FIFO. Note this does *not* reflect how many bytes
    /// are effectively received by the controller.
    pub fn write(&mut self, buf: &[u8]) -> usize {
        // just in case, clears previous tx abort.
        self.i2c.i2c.ic_clr_tx_abrt.read();

        let mut sent = 0;
        for &b in buf.iter() {
            if self.i2c.tx_fifo_full() {
                break;
            }

            self.i2c
                .i2c
                .ic_data_cmd
                .write(|w| unsafe { w.dat().bits(b) });
            sent += 1;
        }
        // serve a pending read request
        self.i2c.i2c.ic_clr_rd_req.read();
        sent
    }

    /// Pull up to `usize::min(RX_FIFO_SIZE, buf.len())` bytes from the RX FIFO.
    pub fn read(&mut self, buf: &mut [u8]) -> usize {
        let mut read = 0;

        for b in buf.iter_mut() {
            if self.i2c.rx_fifo_empty() {
                break;
            }

            *b = self.i2c.i2c.ic_data_cmd.read().dat().bits();
            read += 1;
        }
        read
    }
}
impl<T: Deref<Target = I2CBlock>, PINS> Iterator for I2CPeripheralEventIterator<T, PINS> {
    type Item = I2CEvent;

    fn next(&mut self) -> Option<Self::Item> {
        let stat = self.i2c.i2c.ic_raw_intr_stat.read();
        self.i2c.i2c.ic_clr_activity.read();

        match self.state {
            State::Idle if stat.start_det().bit_is_set() => {
                self.i2c.i2c.ic_clr_start_det.read();
                self.state = State::Active;
                Some(I2CEvent::Start)
            }
            State::Active if !self.i2c.rx_fifo_empty() => {
                self.state = State::Write;
                Some(I2CEvent::TransferWrite)
            }
            State::Active if stat.rd_req().bit_is_set() => {
                // Clearing `rd_req` is used by the hardware to detect when the I2C block can stop
                // stretching the clock and start process the data pushed to the FIFO (if any).
                // This is done in `Self::write`.
                self.state = State::Read;
                Some(I2CEvent::TransferRead)
            }
            State::Read if stat.rd_req().bit_is_set() => Some(I2CEvent::TransferRead),
            State::Read if stat.restart_det().bit_is_set() => {
                self.i2c.i2c.ic_clr_restart_det.read();
                self.state = State::Active;
                Some(I2CEvent::Restart)
            }
            State::Write if !self.i2c.rx_fifo_empty() => Some(I2CEvent::TransferWrite),
            State::Write if stat.restart_det().bit_is_set() => {
                self.i2c.i2c.ic_clr_restart_det.read();
                self.state = State::Active;
                Some(I2CEvent::Restart)
            }
            _ if stat.stop_det().bit_is_set() => {
                self.i2c.i2c.ic_clr_stop_det.read();
                self.state = State::Idle;
                Some(I2CEvent::Stop)
            }
            _ => None,
        }
    }
}

impl<Block, Sda, Scl> I2CPeripheralEventIterator<Block, (Sda, Scl)>
where
    Block: SubsystemReset + Deref<Target = I2CBlock>,
{
    /// Releases the I2C peripheral and associated pins
    #[allow(clippy::type_complexity)]
    pub fn free(self, resets: &mut RESETS) -> (Block, (Sda, Scl)) {
        self.i2c.free(resets)
    }
}
