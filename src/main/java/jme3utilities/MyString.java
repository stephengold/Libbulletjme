/*
 Copyright (c) 2013-2022, Stephen Gold
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

import java.util.Locale;
import java.util.logging.Level;
import java.util.logging.Logger;

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
}
