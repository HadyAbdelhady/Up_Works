import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter/widgets.dart';
import 'package:responsive_dashboard/views/widgets/all_Expensess_and_quick_invoice_section.dart';
import 'package:responsive_dashboard/views/widgets/custom_drawer.dart';
import 'package:responsive_dashboard/views/widgets/income_section.dart';
import 'package:responsive_dashboard/views/widgets/my_cards_and_transaction_history_section.dart';

class DashBoardDesktopLayout extends StatelessWidget {
  const DashBoardDesktopLayout({super.key});

  @override
  Widget build(BuildContext context) {
    return const Row(
      children: [
        Expanded(
          flex: 2,
          child: CustomDrawer(),
        ),
        SizedBox(
          width: 32,
        ),
        Expanded(
          flex: 3,
          child: SingleChildScrollView(
            child: Padding(
              padding: EdgeInsets.only(
                top: 40,
              ),
              child: AllExpensessAndQuickInvoiceSection(),
            ),
          ),
        ),
        SizedBox(
          width: 24,
        ),
        Expanded(
          flex: 3,
          child: SingleChildScrollView(
            child: Column(
              children: [
                SizedBox(
                  height: 40,
                ),
                MyCardsAndTransactionsHistorySection(),
                SizedBox(
                  height: 24,
                ),
                IncomeSection(),
              ],
            ),
          ),
        ),
        SizedBox(
          width: 32,
        ),
      ],
    );
  }
}
