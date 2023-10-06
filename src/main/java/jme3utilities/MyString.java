/*
 Copyright (c) 2013-2023, Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3utilities;

import com.jme3.math.Matrix3f;
import java.util.Locale;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Utility methods for char sequences, strings, and collections of strings.
 *
 * @author Stephen Gold sgold@sonic.net
 */
final public class MyString {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final private static Logger logger
            = Logger.getLogger(MyString.class.getName());
    /**
     * pattern for matching a scientific-notation exponent
     */
    final private static Pattern sciPattern = Pattern.compile("[Ee][+-]?\\d+$");
    /**
     * names of the coordinate axes
     */
    final private static String[] axisNames = {"X", "Y", "Z"};
    // *************************************************************************
    // constructors

    /**
     * A private constructor to inhibit instantiation of this class.
     */
    private MyString() {
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Describe a coordinate axis.
     *
     * @param axisIndex the index of the axis: 0&rarr;X, 1&rarr;Y, 2&rarr;Z
     * @return a textual description (not null, not empty)
     */
    public static String axisName(int axisIndex) {
        Validate.axisIndex(axisIndex, "axis index");
        String axisName = axisNames[axisIndex];
        return axisName;
    }

    /**
     * Generate a textual description of a single-precision floating-point
     * value.
     *
     * @param fValue the value to describe
     * @return a description (not null, not empty)
     */
    public static String describe(float fValue) {
        String raw = String.format(Locale.US, "%g", fValue);
        String result = trimFloat(raw);

        assert result != null;
        assert !result.isEmpty();
        return result;
    }

    /**
     * Generate a textual description of a single-precision floating-point value
     * using at most 3 decimal places.
     *
     * @param fValue the value to describe
     * @return a description (not null, not empty)
     */
    public static String describeFraction(float fValue) {
        String raw = String.format(Locale.US, "%.3f", fValue);
        String result = trimFloat(raw);

        assert result != null;
        assert !result.isEmpty();
        return result;
    }

    /**
     * Generate a textual description of a Matrix3f value.
     *
     * @param matrix the value to describe (may be null, unaffected)
     * @return a description (not null, not empty)
     */
    public static String describeMatrix(Matrix3f matrix) {
        if (matrix == null) {
            return "null";
        }

        StringBuilder result = new StringBuilder(80);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                float element = matrix.get(row, column);
                String desc = describe(element);
                result.append(desc);

                if (row < 2 || column < 2) {
                    result.append(' ');
                }
            }
            if (row < 2) { // Add an extra space between rows.
                result.append(' ');
            }
        }

        return result.toString();
    }

    /**
     * Replace all tab, quote, newline, and backslash characters in the
     * specified text with escape sequences.
     *
     * @param unescaped the input text to escape (not null)
     * @return the escaped text (not null)
     */
    public static String escape(CharSequence unescaped) {
        int length = unescaped.length();
        StringBuilder result = new StringBuilder(length + 10);

        for (int i = 0; i < length; ++i) {
            char ch = unescaped.charAt(i);
            switch (ch) {
                case '\n':
                    result.append("\\n");
                    break;

                case '\t':
                    result.append("\\t");
                    break;

                case '"':
                    result.append("\\\"");
                    break;

                case '\\':
                    result.append("\\\\");
                    break;

                default:
                    result.append(ch);
                    break;
            }
        }

        return result.toString();
    }

    /**
     * Convert the first character of the specified text to lower case.
     *
     * @param input the input text to convert (not null)
     * @return the converted text (not null)
     */
    public static String firstToLower(String input) {
        String result = input;
        if (!input.isEmpty()) {
            String first = input.substring(0, 1);
            first = first.toLowerCase(Locale.ROOT);
            String rest = input.substring(1);
            result = first + rest;
        }

        return result;
    }

    /**
     * Enclose the specified text in quotation marks and escape all tab, quote,
     * newline, and backslash characters.
     *
     * @param text the input text to quote
     * @return the quoted text, or "null" if the input was null
     */
    public static String quote(CharSequence text) {
        String result;
        if (text == null) {
            result = "null";
        } else {
            result = "\"" + escape(text) + "\"";
        }

        return result;
    }

    /**
     * Extract the remainder of the specified string after removing the
     * specified suffix.
     *
     * @param input the input string (not null)
     * @param suffix the suffix string (not null)
     * @return the remainder of the input (not null)
     */
    public static String removeSuffix(String input, String suffix) {
        Validate.nonNull(suffix, "suffix");
        if (!input.endsWith(suffix)) {
            logger.log(Level.SEVERE, "input={0}, suffix={1}", new Object[]{
                quote(input), quote(suffix)
            });
            throw new IllegalArgumentException("input must end with suffix.");
        }

        int endPosition = input.length() - suffix.length();
        String result = input.substring(0, endPosition);

        assert result != null;
        return result;
    }
    // *************************************************************************
    // private methods

    /**
     * Trim any trailing zeros and one trailing decimal point from a string
     * representation of a float. Also remove any leading minus sign from zero.
     *
     * @param input the String to trim (not null)
     * @return a trimmed String (not null)
     */
    private static String trimFloat(String input) {
        String result;
        Matcher matcher = sciPattern.matcher(input);
        if (matcher.find()) {
            int suffixPos = matcher.start();
            String suffix = input.substring(suffixPos);
            String number = input.substring(0, suffixPos);
            result = trimFloat(number) + suffix;

        } else if (input.contains(".")) {
            int end = input.length();
            char[] chars = input.toCharArray();
            while (end >= 1 && chars[end - 1] == '0') {
                --end;
            }
            if (end >= 1 && chars[end - 1] == '.') {
                --end;
            }
            result = input.substring(0, end);

        } else {
            result = input;
        }

        if ("-0".equals(result)) {
            result = "0";
        }

        return result;
    }
}
