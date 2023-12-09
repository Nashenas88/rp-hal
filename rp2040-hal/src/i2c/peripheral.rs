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

use super::{I2CIrq, Peripheral, ValidPinScl, ValidPinSda, I2C};
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
        irqs: &[I2CIrq],
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
                        I2CIrq::StartDet => {
                            w.m_start_det().disabled();
                        }
                        I2CIrq::GenCall => {
                            w.m_gen_call().disabled();
                        }
                        I2CIrq::RdReq => {
                            w.m_rd_req().disabled();
                        }
                        I2CIrq::RestartDet => {
                            w.m_restart_det().disabled();
                        }
                        I2CIrq::RxDone => {
                            w.m_rx_done().disabled();
                        }
                        I2CIrq::RxFull => {
                            w.m_rx_full().disabled();
                        }
                        I2CIrq::RxOver => {
                            w.m_rx_over().disabled();
                        }
                        I2CIrq::RxUnder => {
                            w.m_rx_under().disabled();
                        }
                        I2CIrq::StopDet => {
                            w.m_stop_det().disabled();
                        }
                        I2CIrq::TxAbrt => {
                            w.m_tx_abrt().disabled();
                        }
                        I2CIrq::TxEmpty => {
                            w.m_tx_empty().disabled();
                        }
                        I2CIrq::TxOver => {
                            w.m_tx_over().disabled();
                        }
                        I2CIrq::Activity => {
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
