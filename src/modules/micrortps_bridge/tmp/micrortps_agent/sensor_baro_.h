// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*! 
 * @file sensor_baro_.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _sensor_baro__H_
#define _sensor_baro__H_

// TODO Poner en el contexto.

#include <stdint.h>
#include <array>
#include <string>
#include <vector>

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#define eProsima_user_DllExport __declspec( dllexport )
#else
#define eProsima_user_DllExport
#endif
#else
#define eProsima_user_DllExport
#endif

#if defined(_WIN32)
#if defined(EPROSIMA_USER_DLL_EXPORT)
#if defined(sensor_baro__SOURCE)
#define sensor_baro__DllAPI __declspec( dllexport )
#else
#define sensor_baro__DllAPI __declspec( dllimport )
#endif // sensor_baro__SOURCE
#else
#define sensor_baro__DllAPI
#endif
#else
#define sensor_baro__DllAPI
#endif // _WIN32

namespace eprosima
{
    namespace fastcdr
    {
        class Cdr;
    }
}

/*!
 * @brief This class represents the structure sensor_baro_ defined by the user in the IDL file.
 * @ingroup SENSOR_BARO_
 */
class sensor_baro_
{
public:

    /*!
     * @brief Default constructor.
     */
    eProsima_user_DllExport sensor_baro_();
    
    /*!
     * @brief Default destructor.
     */
    eProsima_user_DllExport ~sensor_baro_();
    
    /*!
     * @brief Copy constructor.
     * @param x Reference to the object sensor_baro_ that will be copied.
     */
    eProsima_user_DllExport sensor_baro_(const sensor_baro_ &x);
    
    /*!
     * @brief Move constructor.
     * @param x Reference to the object sensor_baro_ that will be copied.
     */
    eProsima_user_DllExport sensor_baro_(sensor_baro_ &&x);
    
    /*!
     * @brief Copy assignment.
     * @param x Reference to the object sensor_baro_ that will be copied.
     */
    eProsima_user_DllExport sensor_baro_& operator=(const sensor_baro_ &x);
    
    /*!
     * @brief Move assignment.
     * @param x Reference to the object sensor_baro_ that will be copied.
     */
    eProsima_user_DllExport sensor_baro_& operator=(sensor_baro_ &&x);
    
    /*!
     * @brief This function sets a value in member error_count
     * @param _error_count New value for member error_count
     */
    inline eProsima_user_DllExport void error_count(uint64_t _error_count)
    {
        m_error_count = _error_count;
    }

    /*!
     * @brief This function returns the value of member error_count
     * @return Value of member error_count
     */
    inline eProsima_user_DllExport uint64_t error_count() const
    {
        return m_error_count;
    }

    /*!
     * @brief This function returns a reference to member error_count
     * @return Reference to member error_count
     */
    inline eProsima_user_DllExport uint64_t& error_count()
    {
        return m_error_count;
    }
    /*!
     * @brief This function sets a value in member pressure
     * @param _pressure New value for member pressure
     */
    inline eProsima_user_DllExport void pressure(float _pressure)
    {
        m_pressure = _pressure;
    }

    /*!
     * @brief This function returns the value of member pressure
     * @return Value of member pressure
     */
    inline eProsima_user_DllExport float pressure() const
    {
        return m_pressure;
    }

    /*!
     * @brief This function returns a reference to member pressure
     * @return Reference to member pressure
     */
    inline eProsima_user_DllExport float& pressure()
    {
        return m_pressure;
    }
    /*!
     * @brief This function sets a value in member altitude
     * @param _altitude New value for member altitude
     */
    inline eProsima_user_DllExport void altitude(float _altitude)
    {
        m_altitude = _altitude;
    }

    /*!
     * @brief This function returns the value of member altitude
     * @return Value of member altitude
     */
    inline eProsima_user_DllExport float altitude() const
    {
        return m_altitude;
    }

    /*!
     * @brief This function returns a reference to member altitude
     * @return Reference to member altitude
     */
    inline eProsima_user_DllExport float& altitude()
    {
        return m_altitude;
    }
    /*!
     * @brief This function sets a value in member temperature
     * @param _temperature New value for member temperature
     */
    inline eProsima_user_DllExport void temperature(float _temperature)
    {
        m_temperature = _temperature;
    }

    /*!
     * @brief This function returns the value of member temperature
     * @return Value of member temperature
     */
    inline eProsima_user_DllExport float temperature() const
    {
        return m_temperature;
    }

    /*!
     * @brief This function returns a reference to member temperature
     * @return Reference to member temperature
     */
    inline eProsima_user_DllExport float& temperature()
    {
        return m_temperature;
    }
    /*!
     * @brief This function sets a value in member device_id
     * @param _device_id New value for member device_id
     */
    inline eProsima_user_DllExport void device_id(uint32_t _device_id)
    {
        m_device_id = _device_id;
    }

    /*!
     * @brief This function returns the value of member device_id
     * @return Value of member device_id
     */
    inline eProsima_user_DllExport uint32_t device_id() const
    {
        return m_device_id;
    }

    /*!
     * @brief This function returns a reference to member device_id
     * @return Reference to member device_id
     */
    inline eProsima_user_DllExport uint32_t& device_id()
    {
        return m_device_id;
    }
    
    /*!
     * @brief This function returns the maximum serialized size of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getMaxCdrSerializedSize(size_t current_alignment = 0);

    /*!
     * @brief This function returns the serialized size of a data depending on the buffer alignment.
     * @param data Data which is calculated its serialized size.
     * @param current_alignment Buffer alignment.
     * @return Serialized size.
     */
    eProsima_user_DllExport static size_t getCdrSerializedSize(const sensor_baro_& data, size_t current_alignment = 0);


    /*!
     * @brief This function serializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serialize(eprosima::fastcdr::Cdr &cdr) const;

    /*!
     * @brief This function deserializes an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void deserialize(eprosima::fastcdr::Cdr &cdr);



    /*!
     * @brief This function returns the maximum serialized size of the Key of an object
     * depending on the buffer alignment.
     * @param current_alignment Buffer alignment.
     * @return Maximum serialized size.
     */
    eProsima_user_DllExport static size_t getKeyMaxCdrSerializedSize(size_t current_alignment = 0);

    /*!
     * @brief This function tells you if the Key has been defined for this type
     */
    eProsima_user_DllExport static bool isKeyDefined();

    /*!
     * @brief This function serializes the key members of an object using CDR serialization.
     * @param cdr CDR serialization object.
     */
    eProsima_user_DllExport void serializeKey(eprosima::fastcdr::Cdr &cdr) const;
    
private:
    uint64_t m_error_count;
    float m_pressure;
    float m_altitude;
    float m_temperature;
    uint32_t m_device_id;
};

#endif // _sensor_baro__H_