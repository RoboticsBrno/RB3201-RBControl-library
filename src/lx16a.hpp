#pragma once

#include <stdexcept>
#include <vector>
#include <driver/uart.h>
#include <soc/io_mux_reg.h>
#include <chrono>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Hack
inline esp_err_t gpio_output_disable(gpio_num_t gpio_num)
{
    if (gpio_num < 32) {
        GPIO.enable_w1tc = (0x1 << gpio_num);
    } else {
        GPIO.enable1_w1tc.data = (0x1 << (gpio_num - 32));
    }

    // Ensure no other output signal is routed via GPIO matrix to this pin
    REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (gpio_num * 4),
              SIG_GPIO_OUT_IDX);

    return ESP_OK;
}

class Angle {
public:
    static Angle rad( float r ) { return Angle( 180 * r / M_PI ); }
    static Angle deg( float d ) { return Angle( d ); }

    Angle& operator+=( Angle a ) { _degs += a._degs; return *this; }
    Angle& operator-=( Angle a ) { _degs -= a._degs; return *this; }
    Angle& operator*=( double c ) { _degs *= c; return *this; }
    Angle& operator/=( double c ) { _degs /= c; return *this; }

    float deg() const { return _degs; }
    float rad() const { return _degs / 180.0 * M_PI; }
private:
    float _degs;
    Angle( float d ): _degs( d ) {}
};

Angle operator+( Angle a, Angle b );
Angle operator-( Angle a, Angle b );
Angle operator*( Angle a, float c );
Angle operator/( Angle a, float c );
Angle operator"" _deg ( long double d );
Angle operator"" _rad ( long double r );
Angle operator"" _deg ( unsigned long long int d );
Angle operator"" _rad ( unsigned long long int r );

namespace lw {

int fromDeg( int angle );

enum class Command {
    SERVO_MOVE_TIME_WRITE = 1,
    SERVO_MOVE_TIME_READ,
    SERVO_MOVE_TIME_WAIT_WRITE = 7,
    SERVO_MOVE_TIME_WAIT_READ,
    SERVO_MOVE_START = 11,
    SERVO_MOVE_STOP,
    SERVO_ID_WRITE,
    SERVO_ID_READ,
    SERVO_ANGLE_OFFSET_ADJUST = 17,
    SERVO_ANGLE_OFFSET_WRITE,
    SERVO_ANGLE_OFFSET_READ,
    SERVO_ANGLE_LIMIT_WRITE,
    SERVO_ANGLE_LIMIT_READ,
    SERVO_VIN_LIMIT_WRITE,
    SERVO_VIN_LIMIT_READ,
    SERVO_TEMP_MAX_LIMIT_WRITE,
    SERVO_TEMP_MAX_LIMIT_READ,
    SERVO_TEMP_READ,
    SERVO_VIN_READ,
    SERVO_POS_READ,
    SERVO_OR_MOTOR_MODE_WRITE,
    SERVO_OR_MOTOR_MODE_READ,
    SERVO_LOAD_OR_UNLOAD_WRITE,
    SERVO_LOAD_OR_UNLOAD_READ,
    SERVO_LED_CTRL_WRITE,
    SERVO_LED_CTRL_READ,
    SERVO_LED_ERROR_WRITE,
    SERVO_LED_ERROR_READ
};

using Id = uint8_t;

struct Packet {
    Packet() = default;

    Packet( const uint8_t* data, int len ) {
        for ( int i = 0; i < len; i++ ) {
            _data.push_back( data[ i ] );
        }
    }

    template < typename... Args >
    Packet( Id id, Command c, Args... data ) {
        _buildHeader();
        _data.push_back( id );
        _data.push_back( 3 );
        _data.push_back( static_cast< uint8_t >( c ) );
        _pushData( data... );
        _data.push_back( _checksum( _data ) );
    }

    static Packet move( Id id, uint16_t position, uint16_t time ) {
        return Packet( id, Command::SERVO_MOVE_TIME_WRITE,
            position & 0xFF, position >> 8,
            time && 0xFF, time >> 8 );
    }

    static Packet limitAngle( Id id, uint16_t low, uint16_t high ) {
        return Packet( id, Command::SERVO_ANGLE_LIMIT_WRITE,
            low & 0xFF, low >> 8, high & 0xFF, high >> 8 );
    }

    static Packet setId( Id id, Id newId ) {
        return Packet( id, Command::SERVO_ID_WRITE, newId );
    }

    void _buildHeader() {
        _data.push_back( 0x55 );
        _data.push_back( 0x55 );
    }

    void _pushData() {}

    template < typename... Args >
    void _pushData( uint8_t d, Args... data ) {
        _data.push_back( d );
        _data[ 3 ]++;
        _pushData( data... );
    }

    static uint8_t _checksum( const std::vector< uint8_t >& data,
        int offset = 2, int end_offset = 0 ) {
        uint8_t sum = 0;
        for ( int i = offset; i < data.size() - end_offset; i++ )
            sum += data[ i ];
        return ~sum;
    }

    int size() {
        if ( _data.size() < 4 )
            return -1;
        return _data[ 3 ];
    }

    bool valid() {
        if ( _data.size() < 6 )
            return false;
        uint8_t c = _checksum( _data, 2, 0 );
        if ( c != _data.back() )
            return false;
        if ( size() + 3 != _data.size() )
            return false;
        return true;
    }

    /*void dump() {
        Serial.print("[");
        bool first = true;
        for ( auto x : _data ) {
            if ( !first )
                Serial.print(", ");
            first = false;
            Serial.print( x, HEX );
        }
        Serial.println("]");
    }*/

    std::vector< uint8_t > _data;
};

struct Bus {
    Bus( uart_port_t uart, gpio_num_t pin ) :
        _uart( uart ),
        _pin( pin )
    {
        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        };
        ESP_ERROR_CHECK( uart_param_config( _uart, &uart_config ) );
        _switchToTxMode();
        const int uart_buffer_size = (1024 * 2);
        ESP_ERROR_CHECK(uart_driver_install(_uart, uart_buffer_size, uart_buffer_size,
            10, &_uart_queue, 0));
    }

    struct Servo {
    public:
        // Move servo to given position (in degree) in given time (in milliseconds)
        void move( Angle pos, std::chrono::milliseconds t ) {
            int position = pos.deg();
            int time = t.count();
            if ( position < 0 || position > 240 )
                throw std::runtime_error( "Position out of range" );
            if ( time < 0 )
                throw std::runtime_error( "Time is negative" );
            if ( time > 30000 )
                throw std::runtime_error( "Time is out of range" );
            auto p = Packet::move( _id, fromDeg( position ), time );
            _bus.send( p._data );
        }

        void move( Angle pos ) {
            int position = pos.deg();
            if ( position < 0 || position > 240 )
                throw std::runtime_error( "Position out of range" );
            auto p = Packet::move( _id, fromDeg( position ), 0 );
            _bus.send( p._data );
        }

        // Set limits for the movement
        void limit( Angle b, Angle t ) {
            int bottom = b.deg();
            int top = t.deg();
            if ( bottom < 0 || bottom > 240 )
                throw std::runtime_error( "Bottom limit out of range" );
            if ( top < 0 || top > 240 )
                throw std::runtime_error( "Top limit out of range" );
            auto p = Packet::limitAngle( _id, fromDeg( bottom ), fromDeg( top ) );
            _bus.send( p._data );
        }

        void setId( Id newId ) {
            if ( newId >= 254 )
                throw std::runtime_error( "Invalid ID specified" );
            auto p = Packet::setId( _id, newId );
            _bus.send( p._data );
        }

        friend class Bus;
    private:
        Servo( Id id, Bus& bus ) : _id( id ), _bus( bus ) {}

        Id _id;
        Bus& _bus;
    };

    Servo getServo( Id id ) {
        return Servo( id, *this );
    }

    Servo allServos() {
        return Servo( 254, *this );
    }

    void _switchToTxMode() {
        gpio_output_disable( _pin );
        ESP_ERROR_CHECK (uart_set_pin( _uart, _pin, UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE ) );
    }

    void _switchToRxMode() {
        ESP_ERROR_CHECK (uart_set_pin( _uart, UART_PIN_NO_CHANGE, _pin,
            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE ) );
    }

    void send( const std::vector< uint8_t >& data ) {
        _switchToTxMode();
        auto *buffer = reinterpret_cast< const char * >( data.data() );
        //uart_write_bytes(_uart, buffer, data.size() );
        uart_tx_chars( _uart, buffer, data.size() );
        ESP_ERROR_CHECK( uart_wait_tx_done( _uart, 100 ) );
    }

    Packet receive( int len ) {
        _switchToRxMode();
        uint8_t buff[ 32 ];
        uart_read_bytes( _uart, buff, len, 100 );
        return Packet( buff, len );
    }

    QueueHandle_t _uart_queue;
    uart_port_t _uart;
    gpio_num_t _pin;
};

using Servo = Bus::Servo;

} // namespace lw